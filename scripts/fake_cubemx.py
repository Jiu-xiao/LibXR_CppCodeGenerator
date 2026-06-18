#!/usr/bin/env python3

"""Minimal fake STM32CubeMX runner for local and CI smoke tests."""

from __future__ import annotations

import os
import re
import sys


def _extract_script_path(argv):
    for index, arg in enumerate(argv):
        if arg == "-q" and index + 1 < len(argv):
            return argv[index + 1]
    raise SystemExit("missing -q <script>")


def _strip_quotes(value: str) -> str:
    value = value.strip()
    if len(value) >= 2 and value[0] == '"' and value[-1] == '"':
        return value[1:-1]
    return value


def main() -> int:
    script_path = _extract_script_path(sys.argv[1:])
    with open(script_path, "r", encoding="utf-8") as script_file:
        script = script_file.read()

    if os.environ.get("FAKE_CUBEMX_ECHO_SCRIPT"):
        print(script, end="")

    cwd = os.getcwd()
    target_dir = cwd

    generate_code_match = re.search(r"^generate code\s+(.+)$", script, re.MULTILINE)
    if generate_code_match:
        target_dir = _strip_quotes(generate_code_match.group(1))

    os.makedirs(os.path.join(target_dir, "Core", "Inc"), exist_ok=True)
    os.makedirs(os.path.join(target_dir, "Core", "Src"), exist_ok=True)
    os.makedirs(os.path.join(target_dir, "Drivers"), exist_ok=True)
    os.makedirs(os.path.join(target_dir, "Middlewares"), exist_ok=True)

    with open(os.path.join(target_dir, "Core", "Inc", "fake_generated.h"), "w", encoding="utf-8") as out:
        out.write("// fake cubemx output\n")

    print("FAKE_CUBEMX_OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
