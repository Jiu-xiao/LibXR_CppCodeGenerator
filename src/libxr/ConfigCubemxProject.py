#!/usr/bin/env python

import argparse
import os
import subprocess

def run_command(command):
    """Run a shell command and check the return value."""
    result = subprocess.run(command, shell=True, capture_output=True, text=True)
    if result.returncode == 0:
        print(f"[Pass] Command executed successfully: {command}")
    else:
        print(f"[Error] Command failed: {command}")
        print(result.stderr)
        exit(1)

def find_ioc_file(directory):
    """Search for a .ioc file in the specified directory."""
    for file in os.listdir(directory):
        if file.endswith(".ioc"):
            return os.path.join(directory, file)
    return None

def initialize_git_repository(project_dir):
    """Initialize a Git repository if not already present."""
    git_dir = os.path.join(project_dir, ".git")
    if not os.path.exists(git_dir):
        print(f"[Error] Directory {project_dir} is not a Git repository. Initializing Git...")
        run_command(f"git init {project_dir}")

def create_gitignore_file(project_dir):
    """Create a .gitignore file if it does not exist."""
    gitignore_path = os.path.join(project_dir, ".gitignore")
    if not os.path.exists(gitignore_path):
        print(f"Creating .gitignore file...")
        with open(gitignore_path, "w") as gitignore_file:
            gitignore_file.write("""build/**
.history/**
.cache/**
.config.json
CMakeFiles/**
""")

def add_git_submodule(project_dir):
    """Add the LibXR submodule if not already present."""
    libxr_path = os.path.join(project_dir, "Middlewares/Third_Party/LibXR")
    if not os.path.exists(libxr_path):
        print("[Error] Middlewares/Third_Party/LibXR not found. Adding submodule...")
        run_command(
            f"cd {project_dir} && git submodule add https://github.com/Jiu-Xiao/libxr.git ./Middlewares/Third_Party/LibXR")

def create_user_directory(project_dir):
    """Ensure the User directory exists."""
    user_path = os.path.join(project_dir, "User")
    if not os.path.exists(user_path):
        os.makedirs(user_path)
    return user_path

def process_ioc_file(project_dir, json_output):
    """Parse the .ioc file and generate JSON configuration."""
    print("Parsing .ioc file...")
    run_command(f"xr_parse_ioc -d {project_dir} -o {json_output}")

def generate_cpp_code(json_output, cpp_output):
    """Generate C++ code from JSON configuration."""
    print("Generating C++ code...")
    run_command(f"xr_gen_code -i {json_output} -o {cpp_output}")

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
    parser.add_argument("-c", "--clang", action="store_true", default=None, help="Enable Clang")
    args = parser.parse_args()

    project_dir = args.directory.rstrip("/")
    terminal_source = args.terminal
    clang_enable = True if args.clang is not None else False

    if not os.path.isdir(project_dir):
        print(f"[Error] Directory {project_dir} does not exist")
        exit(1)

    # Initialize Git repository and .gitignore
    initialize_git_repository(project_dir)
    create_gitignore_file(project_dir)

    # Add Git submodule if necessary
    add_git_submodule(project_dir)

    # Find .ioc file
    ioc_file = find_ioc_file(project_dir)
    if not ioc_file:
        print("[Error] No .ioc file found")
        exit(1)

    print(f"Found .ioc file: {ioc_file}")

    # Create user directory
    user_path = create_user_directory(project_dir)

    # Define paths
    json_output = os.path.join(project_dir, ".config.json")
    cpp_output = os.path.join(user_path, "app_main.cpp")

    # Process .ioc file
    process_ioc_file(project_dir, json_output)

    # Generate C++ code
    generate_cpp_code(json_output, cpp_output)

    # Modify STM32 interrupt handlers
    modify_stm32_interrupts(project_dir)

    # Generate CMakeLists.txt with selected compiler
    generate_cmake_file(project_dir, clang_enable)

    # Handle optional terminal source
    if terminal_source:
        print("Modifying terminal device source...")

    print("[Pass] All tasks completed successfully!")

if __name__ == "__main__":
    main()
