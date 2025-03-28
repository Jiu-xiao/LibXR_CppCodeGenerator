#!/usr/bin/env python3

import os
import sys
import subprocess

def main():
    current_dir = os.getcwd()

    # Search for .ioc files in the current directory
    ioc_files = [f for f in os.listdir(current_dir) if f.endswith(".ioc")]
    if not ioc_files:
        print("[ERROR] No .ioc files found in the current directory.")
        sys.exit(1)

    # Build the command: python -m libxr.PeripheralAnalyzerSTM32 -d . <additional args>
    cmd = [
        sys.executable,
        "-m", "libxr.PeripheralAnalyzerSTM32",
        "-d", current_dir,
        *sys.argv[1:]  # Forward all extra args
    ]

    print(f"[INFO] Detected {len(ioc_files)} .ioc file(s). Launching parser...")
    print(f"[CMD] {' '.join(cmd)}")

    subprocess.run(cmd)

if __name__ == "__main__":
    main()
