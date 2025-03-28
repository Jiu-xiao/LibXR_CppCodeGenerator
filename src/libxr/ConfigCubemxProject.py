#!/usr/bin/env python

import argparse
import os
import subprocess
import logging
import sys

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

def run_command(command):
    result = subprocess.run(command, shell=True, capture_output=True, text=True)
    if result.returncode == 0:
        logging.info(f"[OK] {command}")
        return result.stdout
    else:
        logging.error(f"[FAILED] {command}\n{result.stderr}")
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

def add_libxr(project_dir):
    libxr_path = os.path.join(project_dir, "Middlewares", "Third_Party", "LibXR")
    midware_path = os.path.join(project_dir, "Middlewares")
    third_party_path = os.path.join(midware_path, "Third_Party")
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
        run_command(f"cd {project_dir} && git submodule update --init --recursive")
        logging.info("LibXR submodule added and initialized.")
    else:
        logging.info("LibXR submodule already exists. Checking for updates...")
        run_command(f"cd {project_dir} && git submodule update --init --recursive")
        logging.info("LibXR submodule updated.")

def create_user_directory(project_dir):
    """Ensure the User directory exists."""
    user_path = os.path.join(project_dir, "User")
    if not os.path.exists(user_path):
        os.makedirs(user_path)
    return user_path

def process_ioc_file(project_dir, yaml_output):
    """Parse the .ioc file and generate YAML configuration."""
    print("Parsing .ioc file...")
    run_command(f"xr_parse_ioc -d {project_dir} -o {yaml_output}")

def generate_cpp_code(yaml_output, cpp_output, xrobot_enable=False):
    """Generate C++ code from YAML configuration, with optional XRobot support."""
    print("Generating C++ code...")
    cmd = f"xr_gen_code_stm32 -i {yaml_output} -o {cpp_output}"
    if xrobot_enable:
        cmd += " --xrobot"
    run_command(cmd)

def modify_stm32_interrupts(project_dir):
    """Modify STM32 interrupt handler files."""
    print("Modifying STM32 interrupt files...")
    run_command(f"xr_stm32_it {os.path.join(project_dir, 'Core/Src')}")

def generate_cmake_file(project_dir, clang_enable):
    """Generate CMakeLists.txt for STM32 project with selected compiler."""
    run_command(f"xr_stm32_cmake {project_dir}")
    if clang_enable:
        run_command(f"xr_stm32_clang {project_dir}")

def main():
    parser = argparse.ArgumentParser(description="Automate STM32CubeMX project setup")
    parser.add_argument("-d", "--directory", required=True, help="STM32CubeMX project directory")
    parser.add_argument("-t", "--terminal", default="", help="Optional terminal device source")
    parser.add_argument("-c", "--clang", action="store_true", help="Enable Clang")
    parser.add_argument("--xrobot", action="store_true", help="Support XRobot")

    args = parser.parse_args()

    project_dir = args.directory.rstrip("/")
    terminal_source = args.terminal
    clang_enable = bool(args.clang)
    xrobot_enable = bool(args.xrobot)

    if not os.path.isdir(project_dir):
        print(f"[Error] Directory {project_dir} does not exist")
        exit(1)

    # Add Git submodule if necessary
    add_libxr(project_dir)

    # Find .ioc file
    ioc_file = find_ioc_file(project_dir)
    if not ioc_file:
        print("[Error] No .ioc file found")
        exit(1)

    print(f"Found .ioc file: {ioc_file}")

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
        print("Modifying terminal device source...")

    logging.info("[Pass] All tasks completed successfully!")

if __name__ == "__main__":
    main()
