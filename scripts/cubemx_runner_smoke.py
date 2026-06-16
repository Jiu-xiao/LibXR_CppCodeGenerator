#!/usr/bin/env python3

"""Smoke test for the CubeMX script-mode runner using a fake CubeMX executable."""

from __future__ import annotations

import os
import stat
import subprocess
import sys
import tempfile
import textwrap
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = REPO_ROOT / "src"
FAKE_CUBEMX = REPO_ROOT / "scripts" / "fake_cubemx.py"


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


def main() -> int:
    env = os.environ.copy()
    env["PYTHONPATH"] = str(SRC_DIR) + os.pathsep + env.get("PYTHONPATH", "")
    FAKE_CUBEMX.chmod(FAKE_CUBEMX.stat().st_mode | stat.S_IXUSR)

    with tempfile.TemporaryDirectory(prefix="libxr_cubemx_smoke_") as tmpdir:
        project_dir = Path(tmpdir) / "project"
        project_dir.mkdir()
        _write_ioc(project_dir)

        smoke_direct = textwrap.dedent(
            f"""\
            from libxr.CubeMXGenerator import generate_cubemx_project

            result = generate_cubemx_project(
                project_dir=r\"{project_dir}\",
                cubemx_cmd=r\"{FAKE_CUBEMX}\",
                launch_mode=\"direct\",
                java_cmd=\"\",
                expect_paths=[\"Core/Inc/fake_generated.h\", \"Drivers\", \"Middlewares\"],
                script_path=r\"{project_dir / 'manual_script.txt'}\",
                keep_script=True,
            )
            print(result.returncode)
            print(result.script_path)
            """
        )
        subprocess.run([sys.executable, "-c", smoke_direct], check=True, env=env)

        if not (project_dir / "Core" / "Inc" / "fake_generated.h").exists():
            raise SystemExit("direct runner did not create expected output")

        project_cfg = Path(tmpdir) / "cfg_project"
        project_cfg.mkdir()
        _write_ioc(project_cfg)
        (project_cfg / "Core").mkdir()

        cfg_smoke = textwrap.dedent(
            f"""\
            import sys
            from unittest import mock
            import libxr.ConfigCubemxProject as cfg

            calls = []

            def fake_process_ioc_file(project_dir, yaml_output):
                calls.append(("parse", project_dir, yaml_output))
                with open(yaml_output, "w", encoding="utf-8") as out:
                    out.write("mcu: stm32\\n")

            def fake_generate_cpp_code(yaml_output, cpp_output, xrobot_enable=False):
                calls.append(("cpp", yaml_output, cpp_output, xrobot_enable))
                with open(cpp_output, "w", encoding="utf-8") as out:
                    out.write("// fake main\\n")

            def fake_generate_cmake_file(project_dir):
                calls.append(("cmake", project_dir))

            with mock.patch.object(cfg, 'process_ioc_file', side_effect=fake_process_ioc_file), \
                 mock.patch.object(cfg, 'generate_cpp_code', side_effect=fake_generate_cpp_code), \
                 mock.patch.object(cfg, 'generate_cmake_file', side_effect=fake_generate_cmake_file), \
                 mock.patch.object(cfg, 'add_libxr', return_value=None), \
                 mock.patch('libxr.PackageInfo.LibXRPackageInfo.check_and_print', return_value=None):
                sys.argv = [
                    'xr_cubemx_cfg',
                    '-d', r'{project_cfg}',
                    '--cubemx-generate',
                    '--cubemx-cmd', r'{FAKE_CUBEMX}',
                    '--cubemx-launch-mode', 'direct',
                    '--cubemx-expect-path', 'Core/Inc/fake_generated.h',
                ]
                cfg.main()

            assert any(item[0] == 'parse' for item in calls)
            assert any(item[0] == 'cpp' for item in calls)
            assert any(item[0] == 'cmake' for item in calls)
            """
        )
        subprocess.run([sys.executable, "-c", cfg_smoke], check=True, env=env)

    print("CUBEMX_RUNNER_SMOKE_OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
