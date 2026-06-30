#!/usr/bin/env python3
"""Smoke test for stale GPIO device aliases produced from CubeMX pin names."""

from __future__ import annotations

import re
import sys
import tempfile
from pathlib import Path

import yaml


GPIO_PC14_ENTRY_RE = re.compile(
    r"LibXR::Entry<LibXR::GPIO>\(\{\s*PC14\s*,\s*\{(?P<aliases>[^}]*)\}\s*\}\)"
)
STALE_PC14_ENTRY_RE = re.compile(
    r"LibXR::Entry<LibXR::GPIO>\(\{\s*PC14_OSC32_IN\b"
)


def run_stm32_generator(repo_root: Path, config_path: Path, output_path: Path) -> int:
    sys.path.insert(0, str(repo_root / "src"))

    from libxr import GeneratorCodeSTM32  # pylint: disable=import-outside-toplevel

    old_argv = sys.argv[:]
    sys.argv = [
        "libxr.GeneratorCodeSTM32",
        "-i",
        str(config_path),
        "-o",
        str(output_path),
        "--xrobot",
        "--hw-cntr",
    ]
    try:
        try:
            GeneratorCodeSTM32.main()
        except SystemExit as exit_status:
            return exit_status.code if isinstance(exit_status.code, int) else 1
    finally:
        sys.argv = old_argv

    return 0


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

        result = run_stm32_generator(repo_root, config_path, output_path)
        if result != 0:
            return result

        generated = output_path.read_text(encoding="utf-8")
        entry = GPIO_PC14_ENTRY_RE.search(generated)

        if entry is None:
            print("missing merged PC14 alias entry")
            print(generated)
            return 1

        aliases = set(re.findall(r'"([^"]+)"', entry.group("aliases")))
        expected_aliases = {"PC14", "PC14_OSC32_IN"}
        if not expected_aliases.issubset(aliases):
            print(f"PC14 alias entry missing aliases: {sorted(expected_aliases - aliases)}")
            print(generated)
            return 1

        if STALE_PC14_ENTRY_RE.search(generated):
            print("stale PC14_OSC32_IN variable entry was generated")
            print(generated)
            return 1

    print("GPIO alias smoke passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
