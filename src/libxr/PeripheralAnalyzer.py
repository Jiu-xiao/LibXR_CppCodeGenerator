#!/usr/bin/env python3

import logging
import os
import sys
import subprocess
import argparse

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")

def main():
    from libxr.PackageInfo import LibXRPackageInfo

    LibXRPackageInfo.check_and_print()

    parser = argparse.ArgumentParser(description="Run PeripheralAnalyzerSTM32 on a specified directory.")
    parser.add_argument(
        "-d", "--directory",
        required=True,
        help="Input directory containing .ioc files"
    )
    args, extra_args = parser.parse_known_args()

    target_dir = os.path.abspath(args.directory)

    if not os.path.isdir(target_dir):
        logging.error(f"Specified directory does not exist: {target_dir}")
        sys.exit(1)

    # Search for .ioc files in the specified directory
    ioc_files = [f for f in os.listdir(target_dir) if f.endswith(".ioc")]
    if not ioc_files:
        logging.error(f"No .ioc files found in directory: {target_dir}")
        sys.exit(1)

    # Construct the command to run the parser
    cmd = [
        sys.executable,
        "-m", "libxr.PeripheralAnalyzerSTM32",
        "-d", target_dir,
        *extra_args  # Forward other arguments
    ]

    logging.info(f"Detected {len(ioc_files)} .ioc file(s) in '{target_dir}':")
    for f in ioc_files:
        logging.info(f"       - {f}")
    logging.debug(f"CMD: {' '.join(cmd)}")

    try:
        subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError as e:
        logging.error(f"PeripheralAnalyzerSTM32 exited with code {e.returncode}")
        sys.exit(e.returncode)

if __name__ == "__main__":
    main()
