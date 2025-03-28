#!/usr/bin/env python3

import os
import sys
import subprocess
import argparse

def main():
    parser = argparse.ArgumentParser(description="Run PeripheralAnalyzerSTM32 on a specified directory.")
    parser.add_argument(
        "-d", "--directory",
        required=True,
        help="Input directory containing .ioc files"
    )
    args, extra_args = parser.parse_known_args()

    target_dir = os.path.abspath(args.directory)

    if not os.path.isdir(target_dir):
        print(f"[ERROR] Specified directory does not exist: {target_dir}")
        sys.exit(1)

    # Search for .ioc files in the specified directory
    ioc_files = [f for f in os.listdir(target_dir) if f.endswith(".ioc")]
    if not ioc_files:
        print(f"[ERROR] No .ioc files found in directory: {target_dir}")
        sys.exit(1)

    # Construct the command to run the parser
    cmd = [
        sys.executable,
        "-m", "libxr.PeripheralAnalyzerSTM32",
        "-d", target_dir,
        *extra_args  # Forward other arguments
    ]

    print(f"[INFO] Detected {len(ioc_files)} .ioc file(s) in '{target_dir}':")
    for f in ioc_files:
        print(f"       - {f}")
    print(f"[CMD] {' '.join(cmd)}")

    try:
        subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError as e:
        print(f"[ERROR] PeripheralAnalyzerSTM32 exited with code {e.returncode}")
        sys.exit(e.returncode)

if __name__ == "__main__":
    main()
