#!/usr/bin/env python3
"""Smoke test for stale GPIO device aliases produced from CubeMX pin names."""

from __future__ import annotations

import subprocess
import sys
import tempfile
from pathlib import Path

import yaml


def main() -> int:
    repo_root = Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory() as tmp:
        tmp_dir = Path(tmp)
        config_path = tmp_dir / "config.yaml"
        output_path = tmp_dir / "app_main.cpp"
        libxr_config_path = tmp_dir / "libxr_config.yaml"

        config = {
            "Mcu": {"Family": "STM32H7", "Type": "STM32H723VGTx"},
            "Timebase": {"Source": "SysTick"},
            "GPIO": {"PC14": {"Signal": "GPIO_Output"}},
            "Peripherals": {},
            "device_aliases": {
                "PC14_OSC32_IN": {
                    "type": "GPIO",
                    "aliases": ["PC14_OSC32_IN"],
                }
            },
        }
        config_path.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")
        libxr_config_path.write_text("SYSTEM: None\n", encoding="utf-8")

        command = [
            sys.executable,
            "-m",
            "libxr.GeneratorCodeSTM32",
            "-i",
            str(config_path),
            "-o",
            str(output_path),
            "--xrobot",
            "--hw-cntr",
        ]
        result = subprocess.run(
            command,
            cwd=repo_root,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            check=False,
        )
        if result.returncode != 0:
            print(result.stdout)
            return result.returncode

        generated = output_path.read_text(encoding="utf-8")
        required = 'LibXR::Entry<LibXR::GPIO>({PC14, {"PC14", "PC14_OSC32_IN"}})'
        forbidden = "LibXR::Entry<LibXR::GPIO>({PC14_OSC32_IN"

        if required not in generated:
            print("missing merged PC14 alias entry")
            print(generated)
            return 1
        if forbidden in generated:
            print("stale PC14_OSC32_IN variable entry was generated")
            print(generated)
            return 1

    print("GPIO alias smoke passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
