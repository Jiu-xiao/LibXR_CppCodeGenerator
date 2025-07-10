#!/usr/bin/env python

import logging
import os
import subprocess
import sys

import argparse

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")


def is_git_repo(path):
    try:
        result = subprocess.run(
            ["git", "-C", path, "rev-parse", "--is-inside-work-tree"],
            capture_output=True,
            text=True,
            check=True
        )
        return result.stdout.strip() == "true"
    except subprocess.CalledProcessError:
        return False


def is_git_clean(path):
    """Check if the Git repo at `path` has no uncommitted changes."""
    result = subprocess.run(
        ["git", "-C", path, "status", "--porcelain"],
        capture_output=True,
        text=True
    )
    return result.returncode == 0 and result.stdout.strip() == ""


def get_current_branch(path):
    """Get the current branch name of a Git repo at `path`."""
    result = subprocess.run(
        ["git", "-C", path, "rev-parse", "--abbrev-ref", "HEAD"],
        capture_output=True,
        text=True
    )
    return result.stdout.strip()


def run_command(command, ignore_error=False):
    result = subprocess.run(command, shell=True, capture_output=True, text=True)
    if result.returncode == 0:
        logging.info(f"[OK] {command}")
        return result.stdout
    else:
        logging.error(f"[FAILED] {command}\n{result.stderr}")
        if not ignore_error:
            sys.exit(1)


def find_ioc_file(directory):
    """Search for a .ioc file in the specified directory."""
    for file in os.listdir(directory):
        if file.endswith(".ioc"):
            return os.path.join(directory, file)
    return None


def create_gitignore_file(project_dir):
    gitignore_path = os.path.join(project_dir, ".gitignore")
    if not os.path.exists(gitignore_path):
        logging.info("Creating .gitignore file...")
        with open(gitignore_path, "w") as gitignore_file:
            gitignore_file.write("""build/**
.history/**
.cache/**
.config.yaml
CMakeFiles/**
""")


def add_libxr(project_dir, libxr_commit=None):
    libxr_path = os.path.join(project_dir, "Middlewares", "Third_Party", "LibXR")
    midware_path = os.path.join(project_dir, "Middlewares")
    third_party_path = os.path.join(midware_path, "Third_Party")

    try:
        run_command(f"cd {project_dir} && git submodule update --init -- {libxr_path}", ignore_error=True)
    except Exception:
        pass

    if not os.path.exists(midware_path):
        logging.info("Creating Middleware folder...")
        os.makedirs(midware_path)
    if not os.path.exists(third_party_path):
        logging.info("Creating Third Party folder...")
        os.makedirs(third_party_path)

    if not is_git_repo(project_dir):
        logging.warning(f"{project_dir} is not a Git repository. Initializing...")
        run_command(f"git init {project_dir}")

    if not os.path.exists(libxr_path):
        logging.info("Adding LibXR as submodule...")
        run_command(
            f"cd {project_dir} && git submodule add https://github.com/Jiu-Xiao/libxr.git ./Middlewares/Third_Party/LibXR"
        )
        logging.info("LibXR submodule added and initialized.")
    else:
        logging.info("LibXR submodule already exists.")
        if is_git_clean(libxr_path):
            logging.info("LibXR submodule is clean. Fetching latest changes...")
            branch = get_current_branch(libxr_path)
            run_command(f"cd {libxr_path} && git fetch origin", ignore_error=True)
            run_command(f"cd {libxr_path} && git checkout {branch}", ignore_error=True)
            run_command(f"cd {libxr_path} && git pull origin {branch}", ignore_error=True)
            logging.info("LibXR submodule updated to latest remote version.")
            if libxr_commit:
                logging.info(f"Checking out LibXR to locked commit {libxr_commit}")
                run_command(f"cd {libxr_path} && git checkout {libxr_commit}")
        else:
            logging.warning("LibXR submodule has local changes. Skipping update.")


def create_user_directory(project_dir):
    """Ensure the User directory exists."""
    user_path = os.path.join(project_dir, "User")
    if not os.path.exists(user_path):
        os.makedirs(user_path)
    return user_path


def process_ioc_file(project_dir, yaml_output):
    """Parse the .ioc file and generate YAML configuration."""
    logging.info("Parsing .ioc file...")
    run_command(f"xr_parse_ioc -d {project_dir} -o {yaml_output}")


def generate_cpp_code(yaml_output, cpp_output, xrobot_enable=False):
    """Generate C++ code from YAML configuration, with optional XRobot support."""
    logging.info("Generating C++ code...")
    cmd = f"xr_gen_code_stm32 -i {yaml_output} -o {cpp_output}"
    if xrobot_enable:
        cmd += " --xrobot"
    run_command(cmd)


def modify_stm32_interrupts(project_dir):
    """Modify STM32 interrupt handler files."""
    logging.info("Modifying STM32 interrupt files...")
    run_command(f"xr_stm32_it {os.path.join(project_dir, 'Core/Src')}")


def generate_cmake_file(project_dir, clang_enable):
    """Generate CMakeLists.txt for STM32 project with selected compiler."""
    run_command(f"xr_stm32_cmake {project_dir}")
    if clang_enable:
        run_command(f"xr_stm32_clang {project_dir}")


def main():
    from libxr.PackageInfo import LibXRPackageInfo

    LibXRPackageInfo.check_and_print()

    parser = argparse.ArgumentParser(description="Automate STM32CubeMX project setup")
    parser.add_argument("-d", "--directory", required=True, help="STM32CubeMX project directory")
    parser.add_argument("-t", "--terminal", default="", help="Optional terminal device source")
    parser.add_argument("-c", "--clang", action="store_true", help="Enable Clang build support. (DEPRECATED since STM32CubeMX 15.0, now native!)")
    parser.add_argument("--xrobot", action="store_true", help="Support XRobot")
    parser.add_argument("--commit", default="", help="Specify locked LibXR commit hash")

    args = parser.parse_args()

    project_dir = args.directory.rstrip("/")
    terminal_source = args.terminal
    clang_enable = bool(args.clang)
    xrobot_enable = bool(args.xrobot)

    if clang_enable:
        logging.warning(
            "The '-c/--clang' option is deprecated since STM32CubeMX 15.0. Native Clang support is now provided by "
            "CubeMX and this option has no effect.")
        starm_clang_cmake = os.path.join(project_dir, "cmake", "starm-clang.cmake")
        if os.path.exists(starm_clang_cmake):
            logging.error("Detected 'starm-clang.cmake' (CubeMX 15.0+). No clang codegen needed.")
            sys.exit(-1)

    libxr_commit = args.commit.strip()
    if not libxr_commit:
        try:
            sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "src")))
            from libxr.libxr_version import LibXRInfo
            libxr_commit = LibXRInfo.COMMIT
        except Exception as e:
            logging.info(f"No lock commit found in src/libxr/libxr_version.py: {e}")
            libxr_commit = ""

    if (libxr_commit):
        logging.info(f"Locked LibXR commit: {libxr_commit}")

    if not os.path.isdir(project_dir):
        logging.error(f"Directory {project_dir} does not exist")
        exit(1)

    # Add Git submodule if necessary
    add_libxr(project_dir, libxr_commit if libxr_commit else None)

    # Find .ioc file
    ioc_file = find_ioc_file(project_dir)
    if not ioc_file:
        logging.error("No .ioc file found")
        exit(1)

    logging.info(f"Found .ioc file: {ioc_file}")

    # Create user directory
    user_path = create_user_directory(project_dir)

    # Define paths
    yaml_output = os.path.join(project_dir, ".config.yaml")
    cpp_output = os.path.join(user_path, "app_main.cpp")

    # Process .ioc file
    process_ioc_file(project_dir, yaml_output)

    # Generate C++ code
    generate_cpp_code(yaml_output, cpp_output, xrobot_enable)

    # Modify STM32 interrupt handlers
    modify_stm32_interrupts(project_dir)

    # Generate CMakeLists.txt with selected compiler
    generate_cmake_file(project_dir, clang_enable)

    # Handle optional terminal source
    if terminal_source:
        logging.info("Modifying terminal device source...")

    logging.info("[Pass] All tasks completed successfully!")


if __name__ == "__main__":
    main()
