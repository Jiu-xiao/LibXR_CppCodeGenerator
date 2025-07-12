#!/usr/bin/env python3

import logging
import os
import sys
import subprocess
import argparse
from typing import List

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")

def is_stm32_project(path: str) -> bool:
    """Check if the given path contains any .ioc file."""
    try:
        return any(f.endswith(".ioc") for f in os.listdir(path))
    except Exception as e:
        logging.error(f"Cannot check directory '{path}': {e}")
        return False


def main():
    from libxr.PackageInfo import LibXRPackageInfo

    LibXRPackageInfo.check_and_print()
    
    parser = argparse.ArgumentParser(description="Wrapper for STM32 code generation.")
    parser.add_argument("-i", "--input", required=True,
                        help="Input YAML configuration file path")

    # We don't parse all args because we want to forward unknown ones later
    known_args, unknown_args = parser.parse_known_args()

    input_path = os.path.abspath(known_args.input)
    input_dir = os.path.dirname(input_path)

    if not os.path.isfile(input_path):
        logging.error(f"YAML configuration file not found: {input_path}")
        sys.exit(1)

    if not is_stm32_project(input_dir):
        logging.info("Skipped: This is not an STM32 project (no .ioc file found in input file directory).")
        sys.exit(0)

    # Forward all original arguments (not just known) to the generator
    cmd: List[str] = [sys.executable, "-m", "libxr.GeneratorCodeSTM32", *sys.argv[1:]]

    logging.info("STM32 project detected (found .ioc file in input path).")
    logging.debug(f"CMD: {' '.join(cmd)}")

    try:
        subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError as e:
        logging.error(f"Code generation failed with exit code {e.returncode}")
        sys.exit(e.returncode)
    except Exception as e:
        logging.error(f"Unexpected error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
