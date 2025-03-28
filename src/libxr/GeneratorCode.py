#!/usr/bin/env python3

import os
import sys
import subprocess

def is_stm32_project(path: str) -> bool:
    return any(f.endswith(".ioc") for f in os.listdir(path))

def main():
    current_dir = os.getcwd()

    if not is_stm32_project(current_dir):
        print("[INFO] Skipped: This is not an STM32 project (no .ioc file found).")
        sys.exit(0)  # Not an error, just a no-op

    cmd = [sys.executable, "-m", "libxr.GeneratorCodeSTM32", *sys.argv[1:]]

    print("[INFO] STM32 project detected (found .ioc file).")
    print(f"[CMD] {' '.join(cmd)}")

    try:
        subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError as e:
        print(f"[ERROR] Code generation failed with exit code {e.returncode}")
        sys.exit(e.returncode)

if __name__ == "__main__":
    main()
