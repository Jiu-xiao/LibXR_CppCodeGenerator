#!/usr/bin/env python
import argparse
import os
import logging
import shutil

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")

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

file(
  GLOB LIBXR_USER_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")


target_sources(${CMAKE_PROJECT_NAME}
    PRIVATE ${LIBXR_USER_SOURCES}
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

    build_path = os.path.join(input_directory, "build")
    if os.path.exists(build_path) and os.path.isdir(build_path):
        logging.info("Removing existing 'build' directory...")
        shutil.rmtree(build_path)
        logging.info("'build' directory removed.")

    file_path = os.path.join(input_directory, "cmake", "LibXR.CMake")

    if os.path.exists(file_path):
        os.remove(file_path)

    freertos_enable = os.path.exists(os.path.join(input_directory, "Core", "Inc", "FreeRTOSConfig.h"))

    if freertos_enable:
        system = "FreeRTOS"
    else:
        system = "None"

    with open(file_path, "w") as f:
        f.write(file.replace("_LIBXR_SYSTEM_", system))
    logging.info(f"Generated LibXR.CMake at: {file_path}")

    print("LibXR.CMake generated successfully.")

    main_cmake_path = input_directory + "/CMakeLists.txt"
    if os.path.exists(main_cmake_path):
        with open(main_cmake_path, "r") as f:
            cmake_content = f.read()

        if include_cmake_cmd not in cmake_content:
            with open(main_cmake_path, "a") as f:
                f.write('\n# Add LibXR\n' + include_cmake_cmd)
            logging.info("LibXR.CMake included in CMakeLists.txt.")
        else:
            logging.info("LibXR.CMake already included in CMakeLists.txt.")
    else:
        logging.error("CMakeLists.txt not found.")
        exit(1)

