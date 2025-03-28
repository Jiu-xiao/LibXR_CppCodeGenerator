#!/usr/bin/env python3

import os
import sys
import subprocess
import argparse
from typing import List


def is_stm32_project(path: str) -> bool:
    """Check if the given path contains any .ioc file."""
    try:
        return any(f.endswith(".ioc") for f in os.listdir(path))
    except Exception as e:
        print(f"[ERROR] Cannot check directory '{path}': {e}")
        return False


def main():
    parser = argparse.ArgumentParser(description="Wrapper for STM32 code generation.")
    parser.add_argument("-i", "--input", required=True,
                        help="Input YAML configuration file path")

    # We don't parse all args because we want to forward unknown ones later
    known_args, unknown_args = parser.parse_known_args()

    input_path = os.path.abspath(known_args.input)
    input_dir = os.path.dirname(input_path)

    if not os.path.isfile(input_path):
        print(f"[ERROR] YAML configuration file not found: {input_path}")
        sys.exit(1)

    if not is_stm32_project(input_dir):
        print("[INFO] Skipped: This is not an STM32 project (no .ioc file found in input file directory).")
        sys.exit(0)

    # Forward all original arguments (not just known) to the generator
    cmd: List[str] = [sys.executable, "-m", "libxr.GeneratorCodeSTM32", *sys.argv[1:]]

    print("[INFO] STM32 project detected (found .ioc file in input path).")
    print(f"[CMD] {' '.join(cmd)}")

    try:
        subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError as e:
        print(f"[ERROR] Code generation failed with exit code {e.returncode}")
        sys.exit(e.returncode)
    except Exception as e:
        print(f"[ERROR] Unexpected error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
