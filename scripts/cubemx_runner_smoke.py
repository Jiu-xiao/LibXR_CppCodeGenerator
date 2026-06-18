#!/usr/bin/env python3

"""Smoke tests for the CubeMX script-mode runner using a fake CubeMX."""

from __future__ import annotations

import base64
import os
import stat
import sys
import tarfile
import tempfile
import textwrap

from pathlib import Path
from unittest import mock


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = REPO_ROOT / "src"
FAKE_CUBEMX = REPO_ROOT / "scripts" / "fake_cubemx.py"
SECRET_USERNAME = "ci-user@example.invalid"
SECRET_PASSWORD = "not-a-real-secret"
CI_STATE_ENV_KEYS = (
    "HOME",
    "USERPROFILE",
    "APPDATA",
    "LOCALAPPDATA",
    "STM32CUBEMX_CI_STATE_B64",
)

sys.path.insert(0, str(SRC_DIR))

from libxr.CubeMXGenerator import (  # noqa: E402
    GENERIC_DIALOG_CONFIRM_LIMIT,
    _can_use_generic_dialog_fallback,
    _is_dialog_class,
    _is_explicit_dialog_text,
    build_cubemx_command,
    generate_cubemx_project,
    restore_cubemx_ci_state,
)
import libxr.ConfigCubemxProject as cfg  # noqa: E402


def _write_ioc(project_dir: Path) -> None:
    (project_dir / "test.ioc").write_text(
        textwrap.dedent(
            """\
            Mcu.Name=STM32H723VGTx
            ProjectManager.ToolChain=STM32CubeIDE
            ProjectManager.CoupleFile=true
            """
        ),
        encoding="utf-8",
    )


def _run_direct_generation(project_dir: Path) -> None:
    result = generate_cubemx_project(
        project_dir=str(project_dir),
        cubemx_cmd=str(FAKE_CUBEMX),
        launch_mode="direct",
        java_cmd="",
        expect_paths=["Core/Inc/fake_generated.h", "Drivers", "Middlewares"],
        script_path=str(project_dir / "manual_script.txt"),
        keep_script=True,
    )

    if result.returncode != 0:
        raise SystemExit(f"fake CubeMX returned {result.returncode}")
    if not (project_dir / "Core" / "Inc" / "fake_generated.h").exists():
        raise SystemExit("direct runner did not create expected output")


def _run_java_command_home_smoke(tmpdir: Path) -> None:
    saved_env = _save_env(CI_STATE_ENV_KEYS)
    try:
        java_home = tmpdir / "java_home"
        java_home.mkdir()
        _set_fake_user_profile(java_home)
        script_path = str(tmpdir / "cubemx_generate.txt")

        command = build_cubemx_command(
            str(FAKE_CUBEMX),
            script_path,
            launch_mode="java",
            java_cmd=sys.executable,
        )
        jar_index = command.index("-jar")
        expected_home = os.path.abspath(os.path.expanduser("~"))
        expected_prefs = os.path.join(expected_home, ".java")
        if f"-Duser.home={expected_home}" not in command[:jar_index]:
            raise SystemExit("Java CubeMX launch does not bind user.home before -jar")
        if f"-Djava.util.prefs.userRoot={expected_prefs}" not in command[:jar_index]:
            raise SystemExit("Java CubeMX launch does not bind java.util.prefs.userRoot before -jar")
        if command[jar_index + 1:jar_index + 4] != [str(FAKE_CUBEMX), "-q", script_path]:
            raise SystemExit(f"unexpected Java CubeMX command tail: {command}")

        direct_command = build_cubemx_command(str(FAKE_CUBEMX), script_path, launch_mode="direct")
        if any(arg.startswith("-Duser.home=") for arg in direct_command):
            raise SystemExit("direct CubeMX launch unexpectedly received Java user.home options")
    finally:
        _restore_env(saved_env)


def _run_dialog_filter_smoke() -> None:
    if not _is_explicit_dialog_text("Migrate project to the new STM32CubeMX version"):
        raise SystemExit("CubeMX migration dialog text is not recognized")
    if not _is_explicit_dialog_text("Download STM32Cube FW_H7 package and accept license agreement"):
        raise SystemExit("CubeMX package download/license dialog text is not recognized")
    if not _is_explicit_dialog_text("User Preferences"):
        raise SystemExit("CubeMX startup User Preferences window is not recognized")
    if not _is_explicit_dialog_text("Project Manager Settings"):
        raise SystemExit("CubeMX Project Manager Settings window is not recognized")
    if not _is_dialog_class("sunAwtDialog"):
        raise SystemExit("CubeMX Java dialog class is not recognized")

    confirm_counts = {}
    can_fallback = _can_use_generic_dialog_fallback(confirm_counts, 42, "", "sunAwtDialog")
    for _ in range(GENERIC_DIALOG_CONFIRM_LIMIT - 1):
        can_fallback = can_fallback and _can_use_generic_dialog_fallback(confirm_counts, 42, "", "sunAwtDialog")
    if not can_fallback:
        raise SystemExit("generic CubeMX dialog fallback was blocked too early")
    if _can_use_generic_dialog_fallback(confirm_counts, 42, "", "sunAwtDialog"):
        raise SystemExit("generic CubeMX dialog fallback was not bounded")
    if confirm_counts.get(42) != GENERIC_DIALOG_CONFIRM_LIMIT + 1:
        raise SystemExit("generic CubeMX dialog fallback did not suppress repeated limit logs")
    if _can_use_generic_dialog_fallback(confirm_counts, 42, "", "sunAwtDialog"):
        raise SystemExit("generic CubeMX dialog fallback recovered after being suppressed")

    startup_counts = {}
    can_startup_fallback = _can_use_generic_dialog_fallback(startup_counts, 43, "User Preferences", "sunAwtFrame")
    for _ in range(GENERIC_DIALOG_CONFIRM_LIMIT - 1):
        can_startup_fallback = can_startup_fallback and _can_use_generic_dialog_fallback(
            startup_counts,
            43,
            "User Preferences",
            "sunAwtFrame",
        )
    if not can_startup_fallback:
        raise SystemExit("CubeMX startup dialog fallback was blocked too early")
    if _can_use_generic_dialog_fallback(startup_counts, 43, "User Preferences", "sunAwtFrame"):
        raise SystemExit("CubeMX startup dialog fallback was not bounded")
    if startup_counts.get(43) != GENERIC_DIALOG_CONFIRM_LIMIT + 1:
        raise SystemExit("CubeMX startup dialog fallback did not suppress repeated limit logs")


def _run_config_entry(project_dir: Path) -> None:
    calls = []

    def fake_process_ioc_file(project_path, yaml_output):
        calls.append(("parse", project_path, yaml_output))
        with open(yaml_output, "w", encoding="utf-8") as out:
            out.write("mcu: stm32\n")

    def fake_generate_cpp_code(yaml_output, cpp_output, xrobot_enable=False):
        calls.append(("cpp", yaml_output, cpp_output, xrobot_enable))
        with open(cpp_output, "w", encoding="utf-8") as out:
            out.write("// fake main\n")

    def fake_generate_cmake_file(project_path):
        calls.append(("cmake", project_path))

    argv = [
        "xr_cubemx_cfg",
        "-d", str(project_dir),
        "--cubemx-generate",
        "--cubemx-cmd", str(FAKE_CUBEMX),
        "--cubemx-launch-mode", "direct",
        "--cubemx-expect-path", "Core/Inc/fake_generated.h",
    ]

    with mock.patch.object(cfg, "process_ioc_file", side_effect=fake_process_ioc_file), \
         mock.patch.object(cfg, "generate_cpp_code", side_effect=fake_generate_cpp_code), \
         mock.patch.object(cfg, "generate_cmake_file", side_effect=fake_generate_cmake_file), \
         mock.patch.object(cfg, "add_libxr", return_value=None), \
         mock.patch("libxr.PackageInfo.LibXRPackageInfo.check_and_print", return_value=None), \
         mock.patch.object(sys, "argv", argv):
        cfg.main()

    if not any(item[0] == "parse" for item in calls):
        raise SystemExit("xr_cubemx_cfg did not parse IOC after CubeMX generation")
    if not any(item[0] == "cpp" for item in calls):
        raise SystemExit("xr_cubemx_cfg did not generate C++ after CubeMX generation")
    if not any(item[0] == "cmake" for item in calls):
        raise SystemExit("xr_cubemx_cfg did not update CMake after CubeMX generation")


def _make_ci_state_archive(tmpdir: Path) -> Path:
    source_root = tmpdir / "state_src"
    (source_root / ".stm32cubemx").mkdir(parents=True)
    (source_root / "STM32Cube" / "Repository" / "STM32Cube_FW_H7_V1.12.0").mkdir(parents=True)
    (source_root / ".stm32cubemx" / "accepted_firmware_licenses.txt").write_text("FW.H7.1.12.0\n", encoding="utf-8")
    (source_root / "STM32Cube" / "Repository" / "STM32Cube_FW_H7_V1.12.0" / "README.txt").write_text("fake pack\n", encoding="utf-8")

    archive = tmpdir / "cubemx_ci_state.tar.gz"
    with tarfile.open(archive, "w:gz") as tar:
        tar.add(source_root / ".stm32cubemx", arcname=".stm32cubemx")
        tar.add(source_root / "STM32Cube" / "Repository", arcname="STM32Cube/Repository")
    return archive


def _make_unsafe_ci_state_archive(tmpdir: Path) -> Path:
    source = tmpdir / "unsafe_source.txt"
    source.write_text("unsafe\n", encoding="utf-8")
    archive = tmpdir / "unsafe_ci_state.tar.gz"
    with tarfile.open(archive, "w:gz") as tar:
        tar.add(source, arcname="../unsafe.txt")
    return archive


def _save_env(keys):
    return {key: os.environ.get(key) for key in keys}


def _restore_env(saved) -> None:
    for key, value in saved.items():
        if value is None:
            os.environ.pop(key, None)
        else:
            os.environ[key] = value


def _set_fake_user_profile(root: Path) -> None:
    appdata = root / "AppData" / "Roaming"
    local_appdata = root / "AppData" / "Local"
    appdata.mkdir(parents=True, exist_ok=True)
    local_appdata.mkdir(parents=True, exist_ok=True)
    os.environ["HOME"] = str(root)
    os.environ["USERPROFILE"] = str(root)
    os.environ["APPDATA"] = str(appdata)
    os.environ["LOCALAPPDATA"] = str(local_appdata)


def _run_ci_state_restore(tmpdir: Path) -> None:
    archive = _make_ci_state_archive(tmpdir)
    saved_env = _save_env(CI_STATE_ENV_KEYS)
    try:
        restore_home = tmpdir / "restore_home"
        restore_home.mkdir()
        _set_fake_user_profile(restore_home)
        restored = restore_cubemx_ci_state(str(archive))
        if ".stm32cubemx" not in restored or "STM32Cube/Repository" not in restored:
            raise SystemExit(f"unexpected restored targets: {restored}")
        if not (restore_home / ".stm32cubemx" / "accepted_firmware_licenses.txt").exists():
            raise SystemExit("CubeMX user state was not restored")
        if not (restore_home / "STM32Cube" / "Repository" / "STM32Cube_FW_H7_V1.12.0" / "README.txt").exists():
            raise SystemExit("CubeMX repository cache was not restored")

        unsafe_archive = _make_unsafe_ci_state_archive(tmpdir)
        try:
            restore_cubemx_ci_state(str(unsafe_archive))
        except RuntimeError:
            pass
        else:
            raise SystemExit("unsafe CubeMX CI state archive was not rejected")
        if (restore_home.parent / "unsafe.txt").exists():
            raise SystemExit("unsafe archive member escaped the allowed CubeMX state targets")

        b64_home = tmpdir / "restore_home_b64"
        b64_home.mkdir()
        _set_fake_user_profile(b64_home)
        os.environ["STM32CUBEMX_CI_STATE_B64"] = base64.b64encode(archive.read_bytes()).decode("ascii")
        restored_b64 = restore_cubemx_ci_state()
        if "STM32Cube/Repository" not in restored_b64:
            raise SystemExit(f"base64 state archive did not restore repository: {restored_b64}")
    finally:
        _restore_env(saved_env)


def _run_login_secret_smoke(project_dir: Path) -> None:
    old_user = os.environ.get("STM32CUBEMX_USERNAME")
    old_password = os.environ.get("STM32CUBEMX_PASSWORD")
    try:
        os.environ["STM32CUBEMX_USERNAME"] = SECRET_USERNAME
        os.environ["STM32CUBEMX_PASSWORD"] = SECRET_PASSWORD
        log_dir = project_dir / "cubemx_logs"
        os.environ["FAKE_CUBEMX_ECHO_SCRIPT"] = "1"
        generate_cubemx_project(
            project_dir=str(project_dir),
            cubemx_cmd=str(FAKE_CUBEMX),
            launch_mode="direct",
            java_cmd="",
            log_dir=str(log_dir),
            allow_st_login=True,
            timeout=10,
        )
        log_files = [
            log_dir / "cubemx_command.txt",
            log_dir / "cubemx_generate.txt",
            log_dir / "cubemx_stdout.log",
            log_dir / "cubemx_stderr.log",
        ]
        for log_file in log_files:
            log_text = log_file.read_text(encoding="utf-8")
            if SECRET_USERNAME in log_text or SECRET_PASSWORD in log_text:
                raise SystemExit(f"ST login secrets leaked into {log_file.name}")
        script_log = (log_dir / "cubemx_generate.txt").read_text(encoding="utf-8")
        if "login <STM32CUBEMX_USERNAME> <STM32CUBEMX_PASSWORD> true" not in script_log:
            raise SystemExit("redacted CubeMX login command was not written to script log")

        try:
            generate_cubemx_project(
                project_dir=str(project_dir),
                cubemx_cmd=str(FAKE_CUBEMX),
                launch_mode="direct",
                java_cmd="",
                keep_script=True,
                allow_st_login=True,
                timeout=10,
            )
        except RuntimeError as error:
            if "--script-path or --keep-script" not in str(error):
                raise
        else:
            raise SystemExit("persistent CubeMX login script was not rejected")
    finally:
        os.environ.pop("FAKE_CUBEMX_ECHO_SCRIPT", None)
        if old_user is None:
            os.environ.pop("STM32CUBEMX_USERNAME", None)
        else:
            os.environ["STM32CUBEMX_USERNAME"] = old_user
        if old_password is None:
            os.environ.pop("STM32CUBEMX_PASSWORD", None)
        else:
            os.environ["STM32CUBEMX_PASSWORD"] = old_password


def main() -> int:
    FAKE_CUBEMX.chmod(FAKE_CUBEMX.stat().st_mode | stat.S_IXUSR)

    with tempfile.TemporaryDirectory(prefix="libxr_cubemx_smoke_") as tmp:
        tmpdir = Path(tmp)
        project_dir = tmpdir / "project"
        project_dir.mkdir()
        _write_ioc(project_dir)
        _run_direct_generation(project_dir)
        _run_java_command_home_smoke(tmpdir)
        _run_dialog_filter_smoke()

        project_cfg = tmpdir / "cfg_project"
        project_cfg.mkdir()
        _write_ioc(project_cfg)
        (project_cfg / "Core").mkdir()
        _run_config_entry(project_cfg)

        _run_ci_state_restore(tmpdir)

        login_project = tmpdir / "login_project"
        login_project.mkdir()
        _write_ioc(login_project)
        _run_login_secret_smoke(login_project)

    print("CUBEMX_RUNNER_SMOKE_OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
