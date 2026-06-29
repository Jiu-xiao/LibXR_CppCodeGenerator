#!/usr/bin/env python3
"""Smoke tests for the standalone STM32CubeMX runner."""

from __future__ import annotations

import contextlib
import os
import stat
import sys
import tempfile
import textwrap
import time

from pathlib import Path
from typing import Iterator


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = REPO_ROOT / "src"
sys.path.insert(0, str(SRC_DIR))

from libxr.CubeMXGenerator import (
    build_cubemx_command,
    generate_cubemx_project,
)  # noqa: E402


FAKE_CUBEMX = r"""
import os
import subprocess
import sys
import time


def main() -> int:
    if "-q" not in sys.argv:
        print("missing -q", file=sys.stderr)
        return 2

    script_path = sys.argv[sys.argv.index("-q") + 1]
    with open(script_path, "r", encoding="utf-8") as script_file:
        script_text = script_file.read()

    mode = os.environ.get("FAKE_CUBEMX_MODE", "ok")
    if mode == "fail":
        print("fake CubeMX failure", file=sys.stderr)
        return 23

    if mode == "timeout":
        child_pid_file = os.environ.get("FAKE_CUBEMX_CHILD_PID_FILE", "")
        heartbeat_file = os.environ.get("FAKE_CUBEMX_HEARTBEAT_FILE", "")
        child_code = (
            "import pathlib, sys, time\n"
            "heartbeat = pathlib.Path(sys.argv[1])\n"
            "while True:\n"
            "    heartbeat.write_text(str(time.time()), encoding='utf-8')\n"
            "    time.sleep(0.1)\n"
        )
        child = subprocess.Popen([sys.executable, "-c", child_code, heartbeat_file])
        if child_pid_file:
            with open(child_pid_file, "w", encoding="utf-8") as pid_file:
                pid_file.write(str(child.pid))
        for _ in range(50):
            if heartbeat_file and os.path.exists(heartbeat_file):
                break
            time.sleep(0.02)
        time.sleep(30)
        return 0

    if mode == "no-output":
        return 0


    if "project generate" in script_text:
        os.makedirs(os.path.join(os.getcwd(), "Core", "Inc"), exist_ok=True)
        os.makedirs(os.path.join(os.getcwd(), "Drivers"), exist_ok=True)
    else:
        print("unexpected script body", file=sys.stderr)
        return 3

    print("fake CubeMX generated project")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
"""


@contextlib.contextmanager
def patched_env(**values: str) -> Iterator[None]:
    old_values = {key: os.environ.get(key) for key in values}
    try:
        for key, value in values.items():
            if value == "":
                os.environ.pop(key, None)
            else:
                os.environ[key] = value
        yield
    finally:
        for key, value in old_values.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value


def write_fake_cubemx(tmpdir: Path) -> Path:
    fake = tmpdir / "fake_cubemx.py"
    with fake.open("w", encoding="utf-8", newline="\n") as fake_file:
        fake_file.write(f"#!{sys.executable}\n" + textwrap.dedent(FAKE_CUBEMX).lstrip())
    if os.name != "nt":
        fake.chmod(fake.stat().st_mode | stat.S_IXUSR)
    return fake


def write_ioc(project_dir: Path) -> None:
    (project_dir / "demo.ioc").write_text(
        "ProjectManager.ProjectName=demo\n", encoding="utf-8"
    )


def assert_contains(haystack: str, needle: str) -> None:
    if needle not in haystack:
        raise AssertionError(f"expected {needle!r} in {haystack!r}")


def run_command_builder_smoke(tmpdir: Path, fake_cubemx: Path) -> None:
    script_path = str(tmpdir / "cubemx script.txt")
    exe_path = str(tmpdir / "STM32CubeMX.exe")
    jar_path = str(tmpdir / "STM32CubeMX.jar")
    Path(exe_path).write_text("", encoding="utf-8")
    Path(jar_path).write_text("", encoding="utf-8")

    direct = build_cubemx_command(exe_path, script_path, launch_mode="auto")
    if direct != [exe_path, "-q", script_path]:
        raise AssertionError(
            f"auto mode should directly launch executables: {direct!r}"
        )

    java = build_cubemx_command(
        jar_path, script_path, launch_mode="auto", java_cmd=sys.executable
    )
    jar_index = java.index("-jar")
    if java[jar_index + 1 : jar_index + 4] != [jar_path, "-q", script_path]:
        raise AssertionError(
            f"auto mode should launch jars through java -jar: {java!r}"
        )

    try:
        build_cubemx_command(
            exe_path, script_path, launch_mode="java", java_cmd=sys.executable
        )
    except ValueError as error:
        assert_contains(str(error), ".jar")
    else:
        raise AssertionError("java launch mode accepted a non-jar CubeMX path")

    py_cmd = build_cubemx_command(
        str(fake_cubemx), script_path, launch_mode="direct", silent=True
    )
    if py_cmd != [sys.executable, str(fake_cubemx), "-q", script_path, "-s"]:
        raise AssertionError(f"direct launch command changed unexpectedly: {py_cmd!r}")


def run_success_smoke(tmpdir: Path, fake_cubemx: Path) -> None:
    project_dir = tmpdir / "project"
    project_dir.mkdir()
    write_ioc(project_dir)
    log_dir = tmpdir / "logs"

    result = generate_cubemx_project(
        project_dir=str(project_dir),
        cubemx_cmd=str(fake_cubemx),
        launch_mode="direct",
        log_dir=str(log_dir),
        timeout=5,
    )

    if result.returncode != 0:
        raise AssertionError(f"unexpected return code: {result.returncode}")
    if Path(result.script_path).exists():
        raise AssertionError("temporary CubeMX script was not removed")

    script_copy = (log_dir / "cubemx_generate.txt").read_text(encoding="utf-8")
    assert_contains(script_copy, "config load")
    assert_contains(script_copy, "project generate")

    command_log = (log_dir / "cubemx_command.txt").read_text(encoding="utf-8")
    assert_contains(command_log, str(fake_cubemx))
    if (
        not (project_dir / "Core" / "Inc").is_dir()
        or not (project_dir / "Drivers").is_dir()
    ):
        raise AssertionError("fake CubeMX did not create expected output paths")


def run_expect_path_smoke(tmpdir: Path, fake_cubemx: Path) -> None:
    project_dir = tmpdir / "missing-output"
    project_dir.mkdir()
    write_ioc(project_dir)

    with patched_env(FAKE_CUBEMX_MODE="no-output"):
        try:
            generate_cubemx_project(
                project_dir=str(project_dir),
                cubemx_cmd=str(fake_cubemx),
                launch_mode="direct",
                timeout=5,
            )
        except RuntimeError as error:
            assert_contains(str(error), "expected paths")
        else:
            raise AssertionError("missing expected paths did not fail the runner")


def run_returncode_smoke(tmpdir: Path, fake_cubemx: Path) -> None:
    project_dir = tmpdir / "returncode"
    project_dir.mkdir()
    write_ioc(project_dir)

    with patched_env(FAKE_CUBEMX_MODE="fail"):
        try:
            generate_cubemx_project(
                project_dir=str(project_dir),
                cubemx_cmd=str(fake_cubemx),
                launch_mode="direct",
                timeout=5,
            )
        except RuntimeError as error:
            assert_contains(str(error), "exit code 23")
            assert_contains(str(error), "fake CubeMX failure")
        else:
            raise AssertionError("non-zero CubeMX exit did not fail the runner")


def child_stopped_writing(heartbeat_file: Path) -> bool:
    if not heartbeat_file.exists():
        return False
    previous = heartbeat_file.read_text(encoding="utf-8")
    time.sleep(0.5)
    current = heartbeat_file.read_text(encoding="utf-8")
    return current == previous


def run_timeout_smoke(tmpdir: Path, fake_cubemx: Path) -> None:
    project_dir = tmpdir / "timeout"
    project_dir.mkdir()
    write_ioc(project_dir)
    child_pid_file = tmpdir / "child.pid"
    heartbeat_file = tmpdir / "child.heartbeat"

    with patched_env(
        FAKE_CUBEMX_MODE="timeout",
        FAKE_CUBEMX_CHILD_PID_FILE=str(child_pid_file),
        FAKE_CUBEMX_HEARTBEAT_FILE=str(heartbeat_file),
    ):
        try:
            generate_cubemx_project(
                project_dir=str(project_dir),
                cubemx_cmd=str(fake_cubemx),
                launch_mode="direct",
                timeout=3,
            )
        except TimeoutError as error:
            assert_contains(str(error), "timed out")
        else:
            raise AssertionError("timed-out CubeMX process did not fail the runner")

    deadline = time.time() + 3
    while time.time() < deadline:
        if child_stopped_writing(heartbeat_file):
            return
        time.sleep(0.1)
    raise AssertionError(
        "fake CubeMX child process kept running after timeout termination"
    )


def main() -> int:
    with tempfile.TemporaryDirectory(prefix="cubemx_runner_smoke_") as tmp:
        tmpdir = Path(tmp)
        fake_cubemx = write_fake_cubemx(tmpdir)
        run_command_builder_smoke(tmpdir, fake_cubemx)
        run_success_smoke(tmpdir, fake_cubemx)
        run_expect_path_smoke(tmpdir, fake_cubemx)
        run_returncode_smoke(tmpdir, fake_cubemx)
        run_timeout_smoke(tmpdir, fake_cubemx)

    print("CubeMX runner smoke checks passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
