#!/usr/bin/env python

"""STM32CubeMX script-mode generator with optional dialog auto-confirmation."""

from __future__ import annotations

import argparse
import ctypes
import logging
import os
import shutil
import subprocess
import sys
import tempfile
import threading
import time

from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


LOGGER = logging.getLogger(__name__)

POSITIVE_BUTTON_LABELS = (
    "i agree",
    "accept",
    "yes",
    "ok",
    "continue",
    "download",
    "install",
    "finish",
    "close",
    "同意",
    "接受",
    "是",
    "确定",
    "继续",
    "下载",
    "安装",
    "完成",
    "关闭",
)

AGREEMENT_LABELS = (
    "i agree",
    "i accept",
    "accept",
    "agree",
    "同意",
    "接受",
)

DIALOG_KEYWORDS = (
    "migrat",
    "compat",
    "convert",
    "license",
    "agreement",
    "accept",
    "download",
    "install",
    "package",
    "software",
    "firmware",
    "repository",
    "协议",
    "许可",
    "同意",
    "接受",
    "下载",
    "安装",
    "迁移",
    "兼容",
    "转换",
)


def _friendly_path_name(path: str) -> str:
    abs_path = os.path.abspath(path)
    base = os.path.basename(abs_path.rstrip(os.sep))
    return base or abs_path


def _resolve_existing_path(path_or_cmd: str) -> str:
    expanded = os.path.expandvars(os.path.expanduser(path_or_cmd))
    if os.path.exists(expanded):
        return os.path.abspath(expanded)
    found = shutil.which(expanded)
    if found:
        return os.path.abspath(found)
    raise FileNotFoundError(path_or_cmd)


def _iter_cubemx_candidates() -> Iterable[str]:
    env_candidates = (
        os.environ.get("STM32CUBEMX_CMD", ""),
        os.environ.get("CUBEMX_CMD", ""),
        os.environ.get("STM32CUBEMX", ""),
    )
    for value in env_candidates:
        if value:
            yield value

    if os.name == "nt":
        local_appdata = os.environ.get("LOCALAPPDATA", "")
        program_files = os.environ.get("ProgramFiles", "")
        candidates = [
            "STM32CubeMX.exe",
            os.path.join(local_appdata, "Programs", "STM32CubeMX", "STM32CubeMX.exe"),
            os.path.join(program_files, "STMicroelectronics", "STM32Cube", "STM32CubeMX", "STM32CubeMX.exe"),
        ]
    else:
        home = os.path.expanduser("~")
        candidates = [
            "STM32CubeMX",
            os.path.join(home, "STM32CubeMX", "STM32CubeMX"),
            "/opt/st/stm32cubemx/STM32CubeMX",
            "/usr/local/bin/STM32CubeMX",
        ]

    for candidate in candidates:
        yield candidate


def resolve_cubemx_command(explicit_cmd: str = "") -> str:
    if explicit_cmd:
        return _resolve_existing_path(explicit_cmd)

    for candidate in _iter_cubemx_candidates():
        try:
            return _resolve_existing_path(candidate)
        except FileNotFoundError:
            continue

    raise FileNotFoundError(
        "Unable to locate STM32CubeMX. Pass --cubemx-cmd or set STM32CUBEMX_CMD."
    )


def _iter_java_candidates(cubemx_cmd: str) -> Iterable[str]:
    env_candidates = (
        os.environ.get("STM32CUBEMX_JAVA", ""),
        os.environ.get("JAVA_CMD", ""),
    )
    for value in env_candidates:
        if value:
            yield value

    java_home = os.environ.get("JAVA_HOME", "")
    if java_home:
        yield os.path.join(java_home, "bin", "java.exe" if os.name == "nt" else "java")

    cubemx_dir = os.path.dirname(os.path.abspath(cubemx_cmd))
    bundled_java = os.path.join(cubemx_dir, "jre", "bin", "java.exe" if os.name == "nt" else "java")
    yield bundled_java
    yield "java"


def resolve_java_command(cubemx_cmd: str, java_cmd: str = "") -> str:
    if java_cmd:
        return _resolve_existing_path(java_cmd)

    for candidate in _iter_java_candidates(cubemx_cmd):
        try:
            return _resolve_existing_path(candidate)
        except FileNotFoundError:
            continue

    raise FileNotFoundError(
        "Unable to locate Java runtime for STM32CubeMX. Pass --java-cmd or use --launch-mode direct."
    )


def _format_script_path(path: str) -> str:
    normalized = os.path.abspath(path)
    if os.name == "nt":
        normalized = normalized.replace("\\", "/")
    if any(ch.isspace() for ch in normalized):
        return f'"{normalized}"'
    return normalized


def build_cubemx_script(ioc_path: str, generate_code_dir: str = "") -> str:
    script_lines = [f"config load {_format_script_path(ioc_path)}"]
    if generate_code_dir:
        script_lines.append(f"generate code {_format_script_path(generate_code_dir)}")
    else:
        script_lines.append("project generate")
    script_lines.append("exit")
    return "\n".join(script_lines) + "\n"


def _shell_join(args: Sequence[str]) -> str:
    try:
        import shlex
        return " ".join(shlex.quote(arg) for arg in args)
    except Exception:
        return " ".join(args)


def build_cubemx_command(
    cubemx_cmd: str,
    script_path: str,
    launch_mode: str = "auto",
    java_cmd: str = "",
    silent: bool = False,
) -> List[str]:
    launch_mode = launch_mode.lower()
    if launch_mode not in {"auto", "direct", "java"}:
        raise ValueError(f"Unsupported launch mode: {launch_mode}")

    use_java = launch_mode == "java" or (launch_mode == "auto" and os.name == "nt")

    command: List[str]
    if use_java:
        resolved_java = resolve_java_command(cubemx_cmd, java_cmd)
        command = [resolved_java, "-jar", cubemx_cmd, "-q", script_path]
    elif cubemx_cmd.lower().endswith(".py"):
        command = [sys.executable, cubemx_cmd, "-q", script_path]
    else:
        command = [cubemx_cmd, "-q", script_path]

    if silent:
        command.append("-s")
    return command


def find_ioc_file(directory: str) -> Optional[str]:
    for file_name in sorted(os.listdir(directory)):
        if file_name.endswith(".ioc"):
            return os.path.join(directory, file_name)
    return None


@dataclass
class CubeMXRunResult:
    command: List[str]
    script_path: str
    stdout: str
    stderr: str
    returncode: int
    log_dir: str = ""


class _BaseDialogController:
    def pump_once(self) -> None:
        raise NotImplementedError


class _NullDialogController(_BaseDialogController):
    def pump_once(self) -> None:
        return


class _WindowsDialogController(_BaseDialogController):
    BM_CLICK = 0x00F5
    BM_GETCHECK = 0x00F0
    BST_CHECKED = 0x0001
    VK_RETURN = 0x0D
    WM_KEYDOWN = 0x0100
    WM_KEYUP = 0x0101

    def __init__(self, process_id: int):
        from ctypes import wintypes

        self.process_id = process_id
        self.wintypes = wintypes
        self.user32 = ctypes.windll.user32
        self._last_action: Dict[int, float] = {}

    def pump_once(self) -> None:
        hwnds = self._enum_windows()
        for hwnd in hwnds:
            title = self._window_text(hwnd)
            child_items = self._child_items(hwnd)
            flat_text = "\n".join([title] + [text for _, text in child_items]).lower()
            if not self._looks_relevant(flat_text):
                continue
            if self._acted_recently(hwnd):
                continue
            if self._accept_window(hwnd, child_items):
                self._last_action[hwnd] = time.time()

    def _enum_windows(self) -> List[int]:
        hwnds: List[int] = []
        enum_proc = ctypes.WINFUNCTYPE(ctypes.c_bool, self.wintypes.HWND, self.wintypes.LPARAM)

        def callback(hwnd: int, _lparam: int) -> bool:
            if not self.user32.IsWindowVisible(hwnd):
                return True
            pid = self.wintypes.DWORD()
            self.user32.GetWindowThreadProcessId(hwnd, ctypes.byref(pid))
            if pid.value == self.process_id:
                hwnds.append(hwnd)
            return True

        self.user32.EnumWindows(enum_proc(callback), 0)
        return hwnds

    def _child_items(self, hwnd: int) -> List[Tuple[int, str]]:
        items: List[Tuple[int, str]] = []
        enum_proc = ctypes.WINFUNCTYPE(ctypes.c_bool, self.wintypes.HWND, self.wintypes.LPARAM)

        def callback(child_hwnd: int, _lparam: int) -> bool:
            items.append((child_hwnd, self._window_text(child_hwnd)))
            return True

        self.user32.EnumChildWindows(hwnd, enum_proc(callback), 0)
        return items

    def _window_text(self, hwnd: int) -> str:
        length = self.user32.GetWindowTextLengthW(hwnd)
        if length <= 0:
            return ""
        buffer = ctypes.create_unicode_buffer(length + 1)
        self.user32.GetWindowTextW(hwnd, buffer, length + 1)
        return buffer.value.strip()

    def _class_name(self, hwnd: int) -> str:
        buffer = ctypes.create_unicode_buffer(256)
        self.user32.GetClassNameW(hwnd, buffer, len(buffer))
        return buffer.value

    def _looks_relevant(self, flat_text: str) -> bool:
        return any(keyword in flat_text for keyword in DIALOG_KEYWORDS)

    def _acted_recently(self, hwnd: int) -> bool:
        last = self._last_action.get(hwnd, 0.0)
        return (time.time() - last) < 2.0

    def _accept_window(self, hwnd: int, child_items: Sequence[Tuple[int, str]]) -> bool:
        for child_hwnd, text in child_items:
            lowered = text.lower()
            if not any(label in lowered for label in AGREEMENT_LABELS):
                continue
            if self._class_name(child_hwnd) != "Button":
                continue
            checked = self.user32.SendMessageW(child_hwnd, self.BM_GETCHECK, 0, 0)
            if checked != self.BST_CHECKED:
                self.user32.SendMessageW(child_hwnd, self.BM_CLICK, 0, 0)
                LOGGER.info("Auto-confirmed agreement checkbox: %s", text)

        for child_hwnd, text in child_items:
            lowered = text.lower()
            if any(label in lowered for label in POSITIVE_BUTTON_LABELS):
                self.user32.SendMessageW(child_hwnd, self.BM_CLICK, 0, 0)
                LOGGER.info("Auto-confirmed CubeMX dialog button: %s", text)
                return True

        self.user32.SetForegroundWindow(hwnd)
        self.user32.PostMessageW(hwnd, self.WM_KEYDOWN, self.VK_RETURN, 0)
        self.user32.PostMessageW(hwnd, self.WM_KEYUP, self.VK_RETURN, 0)
        LOGGER.info("Auto-confirmed CubeMX dialog via Enter fallback")
        return True


class _LinuxX11DialogController(_BaseDialogController):
    def __init__(self, process_id: int):
        from Xlib import X, XK, display  # type: ignore
        from Xlib.ext import xtest  # type: ignore

        self.X = X
        self.XK = XK
        self.display_module = display
        self.xtest = xtest
        self.process_id = process_id
        self.display = display.Display()
        self.root = self.display.screen().root
        self.pid_atom = self.display.intern_atom("_NET_WM_PID")
        self.name_atom = self.display.intern_atom("_NET_WM_NAME")
        self.utf8_atom = self.display.intern_atom("UTF8_STRING")
        self._last_action: Dict[int, float] = {}

    def pump_once(self) -> None:
        for window in self._iter_windows(self.root):
            if self._window_pid(window) != self.process_id:
                continue
            title = self._window_title(window).lower()
            if not self._looks_relevant(title):
                continue
            if self._acted_recently(window.id):
                continue
            self._activate_window(window)
            self._confirm_window()
            self._last_action[window.id] = time.time()

    def _iter_windows(self, window):
        yield window
        try:
            children = window.query_tree().children
        except Exception:
            return
        for child in children:
            yield from self._iter_windows(child)

    def _window_pid(self, window) -> int:
        try:
            prop = window.get_full_property(self.pid_atom, self.X.AnyPropertyType)
            if prop and prop.value:
                return int(prop.value[0])
        except Exception:
            return -1
        return -1

    def _window_title(self, window) -> str:
        try:
            prop = window.get_full_property(self.name_atom, self.utf8_atom)
            if prop and prop.value:
                value = prop.value
                if isinstance(value, bytes):
                    return value.decode("utf-8", errors="ignore")
                return str(value)
        except Exception:
            pass
        try:
            name = window.get_wm_name()
            return name or ""
        except Exception:
            return ""

    def _looks_relevant(self, flat_text: str) -> bool:
        return any(keyword in flat_text for keyword in DIALOG_KEYWORDS) or "stm32cubemx" in flat_text

    def _acted_recently(self, window_id: int) -> bool:
        last = self._last_action.get(window_id, 0.0)
        return (time.time() - last) < 3.0

    def _activate_window(self, window) -> None:
        try:
            window.set_input_focus(self.X.RevertToParent, self.X.CurrentTime)
            self.display.sync()
        except Exception:
            pass

    def _tap(self, key_name: str, alt: bool = False) -> None:
        keycode = self.display.keysym_to_keycode(self.XK.string_to_keysym(key_name))
        if not keycode:
            return
        altcode = self.display.keysym_to_keycode(self.XK.string_to_keysym("Alt_L"))
        if alt and altcode:
            self.xtest.fake_input(self.display, self.X.KeyPress, altcode)
        self.xtest.fake_input(self.display, self.X.KeyPress, keycode)
        self.xtest.fake_input(self.display, self.X.KeyRelease, keycode)
        if alt and altcode:
            self.xtest.fake_input(self.display, self.X.KeyRelease, altcode)
        self.display.sync()
        time.sleep(0.05)

    def _confirm_window(self) -> None:
        for key_name, alt in (("space", False), ("Tab", False), ("Return", False), ("y", True), ("o", True), ("i", True), ("a", True)):
            self._tap(key_name, alt=alt)
        LOGGER.info("Auto-confirmed CubeMX dialog with X11 key sequence")


def create_dialog_controller(process_id: int) -> _BaseDialogController:
    if os.name == "nt":
        return _WindowsDialogController(process_id)
    if not os.environ.get("DISPLAY"):
        LOGGER.warning("CubeMX auto-confirm is enabled but DISPLAY is not set; dialog automation is disabled.")
        return _NullDialogController()
    try:
        return _LinuxX11DialogController(process_id)
    except ImportError:
        LOGGER.warning(
            "CubeMX auto-confirm on Linux requires python-xlib. Install it or disable --auto-confirm."
        )
        return _NullDialogController()
    except Exception as error:
        LOGGER.warning("CubeMX auto-confirm could not start on Linux: %s", error)
        return _NullDialogController()


class _DialogWatchThread(threading.Thread):
    def __init__(self, process_id: int, stop_event: threading.Event, poll_interval: float = 0.5):
        super().__init__(daemon=True)
        self.controller = create_dialog_controller(process_id)
        self.stop_event = stop_event
        self.poll_interval = poll_interval

    def run(self) -> None:
        while not self.stop_event.is_set():
            try:
                self.controller.pump_once()
            except Exception as error:
                LOGGER.warning("CubeMX dialog watcher error: %s", error)
            self.stop_event.wait(self.poll_interval)


def _tail_text(text: str, lines: int = 40) -> str:
    text_lines = text.splitlines()
    return "\n".join(text_lines[-lines:])


def _write_text_file(path: str, content: str) -> None:
    with open(path, "w", encoding="utf-8", newline="\n") as file:
        file.write(content)


def _prepare_script_path(project_dir: str, script_path: str, keep_script: bool) -> Tuple[str, bool]:
    if script_path:
        return os.path.abspath(script_path), False
    if keep_script:
        return os.path.join(project_dir, "cubemx_generate.txt"), False

    handle = tempfile.NamedTemporaryFile(
        prefix="cubemx_generate_",
        suffix=".txt",
        dir=project_dir,
        delete=False,
    )
    handle.close()
    return handle.name, True


def _normalize_expect_paths(project_dir: str, expect_paths: Sequence[str]) -> List[str]:
    resolved = []
    for path in expect_paths:
        if os.path.isabs(path):
            resolved.append(os.path.abspath(path))
        else:
            resolved.append(os.path.abspath(os.path.join(project_dir, path)))
    return resolved


def generate_cubemx_project(
    project_dir: str,
    ioc_file: str = "",
    cubemx_cmd: str = "",
    java_cmd: str = "",
    launch_mode: str = "auto",
    generate_code_dir: str = "",
    expect_paths: Optional[Sequence[str]] = None,
    log_dir: str = "",
    script_path: str = "",
    keep_script: bool = False,
    silent: bool = False,
    auto_confirm: bool = False,
    timeout: int = 1200,
) -> CubeMXRunResult:
    project_dir = os.path.abspath(project_dir)
    if not os.path.isdir(project_dir):
        raise FileNotFoundError(f"Project directory not found: {project_dir}")

    ioc_path = os.path.abspath(ioc_file) if ioc_file else find_ioc_file(project_dir)
    if not ioc_path:
        raise FileNotFoundError(f"No .ioc file found in {_friendly_path_name(project_dir)}")

    resolved_cubemx_cmd = resolve_cubemx_command(cubemx_cmd)
    actual_script_path, should_cleanup_script = _prepare_script_path(project_dir, script_path, keep_script)
    _write_text_file(actual_script_path, build_cubemx_script(ioc_path, generate_code_dir))

    if log_dir:
        os.makedirs(log_dir, exist_ok=True)
        _write_text_file(os.path.join(log_dir, "cubemx_generate.txt"), build_cubemx_script(ioc_path, generate_code_dir))

    command = build_cubemx_command(
        resolved_cubemx_cmd,
        actual_script_path,
        launch_mode=launch_mode,
        java_cmd=java_cmd,
        silent=silent,
    )
    LOGGER.info("Running CubeMX command: %s", _shell_join(command))

    stdout_lines: List[str] = []
    stderr_lines: List[str] = []

    stdout_handle = None
    stderr_handle = None
    if log_dir:
        stdout_handle = open(os.path.join(log_dir, "cubemx_stdout.log"), "w", encoding="utf-8", newline="\n")
        stderr_handle = open(os.path.join(log_dir, "cubemx_stderr.log"), "w", encoding="utf-8", newline="\n")
        _write_text_file(os.path.join(log_dir, "cubemx_command.txt"), _shell_join(command) + "\n")

    def consume_stream(stream, sink: List[str], handle) -> None:
        try:
            for line in iter(stream.readline, ""):
                sink.append(line)
                if handle is not None:
                    handle.write(line)
                    handle.flush()
        finally:
            stream.close()

    process = subprocess.Popen(
        command,
        cwd=project_dir,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        errors="replace",
        bufsize=1,
    )

    stop_event = threading.Event()
    watch_thread = None
    if auto_confirm:
        watch_thread = _DialogWatchThread(process.pid, stop_event)
        watch_thread.start()

    stdout_thread = threading.Thread(target=consume_stream, args=(process.stdout, stdout_lines, stdout_handle), daemon=True)
    stderr_thread = threading.Thread(target=consume_stream, args=(process.stderr, stderr_lines, stderr_handle), daemon=True)
    stdout_thread.start()
    stderr_thread.start()

    timeout_error: Optional[TimeoutError] = None
    try:
        returncode = process.wait(timeout=timeout)
    except subprocess.TimeoutExpired as error:
        process.kill()
        returncode = process.wait(timeout=5)
        timeout_error = TimeoutError(f"STM32CubeMX timed out after {timeout} seconds")
        timeout_error.__cause__ = error
    finally:
        stop_event.set()
        if watch_thread is not None:
            watch_thread.join(timeout=2.0)

    stdout_thread.join(timeout=2.0)
    stderr_thread.join(timeout=2.0)

    stdout_text = "".join(stdout_lines)
    stderr_text = "".join(stderr_lines)

    if stdout_handle is not None:
        stdout_handle.close()
    if stderr_handle is not None:
        stderr_handle.close()

    result = CubeMXRunResult(
        command=command,
        script_path=actual_script_path,
        stdout=stdout_text,
        stderr=stderr_text,
        returncode=returncode,
        log_dir=os.path.abspath(log_dir) if log_dir else "",
    )

    if should_cleanup_script:
        try:
            os.remove(actual_script_path)
        except OSError:
            pass

    if timeout_error is not None:
        raise timeout_error

    if returncode != 0:
        stdout_tail = _tail_text(stdout_text)
        stderr_tail = _tail_text(stderr_text)
        raise RuntimeError(
            "STM32CubeMX generation failed with exit code "
            f"{returncode}\nSTDOUT tail:\n{stdout_tail}\nSTDERR tail:\n{stderr_tail}"
        )

    missing_paths = [path for path in _normalize_expect_paths(project_dir, expect_paths or []) if not os.path.exists(path)]
    if missing_paths:
        raise RuntimeError(
            "STM32CubeMX finished but expected paths are still missing: " + ", ".join(missing_paths)
        )

    LOGGER.info("STM32CubeMX generation finished successfully.")
    return result


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")

    parser = argparse.ArgumentParser(description="Generate STM32CubeMX projects in script mode")
    parser.add_argument("-d", "--directory", required=True, help="Directory containing the CubeMX .ioc file")
    parser.add_argument("--ioc", default="", help="Explicit .ioc file path (defaults to the first .ioc in --directory)")
    parser.add_argument("--cubemx-cmd", default="", help="STM32CubeMX executable path")
    parser.add_argument("--java-cmd", default="", help="Java executable path for -jar launch mode")
    parser.add_argument(
        "--launch-mode",
        choices=("auto", "direct", "java"),
        default="auto",
        help="CubeMX launch mode (default: auto, Windows prefers java -jar)",
    )
    parser.add_argument("--generate-code-dir", default="", help="Use 'generate code <dir>' instead of 'project generate'")
    parser.add_argument("--expect-path", action="append", default=[], help="Path that must exist after generation")
    parser.add_argument("--log-dir", default="", help="Optional directory for command/script/stdout/stderr logs")
    parser.add_argument("--script-path", default="", help="Optional path for the generated CubeMX script file")
    parser.add_argument("--keep-script", action="store_true", help="Keep the generated CubeMX script in the project directory")
    parser.add_argument("--silent", action="store_true", help="Pass -s to STM32CubeMX")
    parser.add_argument("--auto-confirm", action="store_true", help="Attempt to auto-confirm migration/license/download dialogs")
    parser.add_argument("--timeout", type=int, default=1200, help="CubeMX process timeout in seconds (default: 1200)")

    args = parser.parse_args()

    try:
        generate_cubemx_project(
            project_dir=args.directory,
            ioc_file=args.ioc,
            cubemx_cmd=args.cubemx_cmd,
            java_cmd=args.java_cmd,
            launch_mode=args.launch_mode,
            generate_code_dir=args.generate_code_dir,
            expect_paths=args.expect_path,
            log_dir=args.log_dir,
            script_path=args.script_path,
            keep_script=args.keep_script,
            silent=args.silent,
            auto_confirm=args.auto_confirm,
            timeout=args.timeout,
        )
    except Exception as error:
        LOGGER.error("%s", error)
        sys.exit(1)


if __name__ == "__main__":
    main()
