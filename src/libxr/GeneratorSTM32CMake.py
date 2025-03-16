#!/usr/bin/env python
import argparse
import fnmatch
import os
import re

file =(
'''set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# LibXR
set(LIBXR_SYSTEM _LIBXR_SYSTEM_)
set(LIBXR_DRIVER st)
add_subdirectory(Middlewares/Third_Party/LibXR)
target_link_libraries(
    xr
    stm32cubemx
)

target_include_directories(xr
    PUBLIC $<TARGET_PROPERTY:stm32cubemx,INTERFACE_INCLUDE_DIRECTORIES>
    PUBLIC Core/Inc
    PUBLIC User
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    PUBLIC $<TARGET_PROPERTY:xr,INTERFACE_INCLUDE_DIRECTORIES>
    PUBLIC User
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
    xr
)


target_sources(${CMAKE_PROJECT_NAME}
    PRIVATE User/app_main.cpp
)
'''
)

include_cmake_cmd = "include(${CMAKE_CURRENT_LIST_DIR}/cmake/LibXR.CMake)\n"


def main():
    parser = argparse.ArgumentParser(description="Generate CMake file for LibXR.")
    parser.add_argument("input_dir", type=str, help="CubeMX CMake Project Directory")

    args = parser.parse_args()
    input_directory = args.input_dir

    if not os.path.isdir(input_directory):
        print("Input directory does not exist.")
        exit(1)

    file_path = input_directory + "/cmake/LibXR.CMake"
    if os.path.exists(file_path):
        os.remove(file_path)

    if os.path.exists(input_directory + "/Core/Inc/FreeRTOSConfig.h"):
        freertos_enable = True
    else:
        freertos_enable = False


    if freertos_enable:
        system = "FreeRTOS"
    else:
        system = "None"

    with open(file_path, "w") as f:
        f.write(file.replace("_LIBXR_SYSTEM_", system))
        f.close()

    print("LibXR.CMake generated successfully.")

    main_cmake_path = input_directory + "/CMakeLists.txt"
    if os.path.exists(main_cmake_path):
        if include_cmake_cmd not in open(main_cmake_path).read():
            with open(main_cmake_path, "a") as f:
                f.write('\n# Add LibXR\n' + include_cmake_cmd)
                f.close()
            print("LibXR.CMake included in CMakeLists.txt.")
        else:
            print("LibXR.CMake already included in CMakeLists.txt.")
    else:
        print("Error: CMakeLists.txt not found.")
        exit(1)
