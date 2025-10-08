#!/usr/bin/env python
import argparse
import os
import logging
import shutil
from pathlib import Path
from typing import Union

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")

LIBXR_CMAKE_TEMPLATE = (
'''set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# LibXR
set(LIBXR_SYSTEM _LIBXR_SYSTEM_)
set(LIBXR_DRIVER st)
set(XROBOT_MODULES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Modules)
add_subdirectory(Middlewares/Third_Party/LibXR)
target_link_libraries(xr
    PUBLIC stm32cubemx
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
    GLOB LIBXR_USER_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/User/*.cpp")


target_sources(${CMAKE_PROJECT_NAME}
    PRIVATE ${LIBXR_USER_SOURCES}
)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE -Og)
    target_compile_options(xr PRIVATE -O2)
    if(TARGET FreeRTOS)
        target_compile_options(FreeRTOS PRIVATE -O2)
    endif()

    if(TARGET STM32_Drivers)
        target_compile_options(STM32_Drivers PRIVATE -O2)
    endif()

    if(TARGET USB_Device_Library)
        target_compile_options(USB_Device_Library PRIVATE -O2)
    endif()
endif()
'''
)

include_cmake_cmd = "include(${CMAKE_CURRENT_LIST_DIR}/cmake/LibXR.CMake)\n"


def clean_cmake_build_dirs(input_directory: Union[str, Path]) -> None:
    input_directory = Path(input_directory)
    removed = False
    for d in input_directory.iterdir():
        if d.is_dir() and (d.name == "build" or d.name.startswith("cmake-build")):
            shutil.rmtree(d)
            logging.info(f"Removed {d}")
            removed = True
    if not removed:
        logging.info("No build or cmake-build* directory found, nothing to clean.")


def read_text_with_fallback(path: str) -> str:
    for enc in ("utf-8", "utf-8-sig", "gb18030"):
        try:
            return Path(path).read_text(encoding=enc)
        except UnicodeDecodeError:
            continue
    return Path(path).read_text(encoding="utf-8")


def main():
    from libxr.PackageInfo import LibXRPackageInfo

    LibXRPackageInfo.check_and_print()

    parser = argparse.ArgumentParser(description="Generate CMake file for LibXR.")
    parser.add_argument("input_dir", type=str, help="CubeMX CMake Project Directory")

    args = parser.parse_args()
    input_directory = args.input_dir

    if not os.path.isdir(input_directory):
        logging.error("Input directory does not exist.")
        exit(1)

    clean_cmake_build_dirs(input_directory)

    cmake_dir = os.path.join(input_directory, "cmake")
    os.makedirs(cmake_dir, exist_ok=True)

    file_path = os.path.join(cmake_dir, "LibXR.CMake")

    if os.path.exists(file_path):
        os.remove(file_path)

    freertos_enable = os.path.exists(os.path.join(input_directory, "Core", "Inc", "FreeRTOSConfig.h"))
    threadx_enable = os.path.exists(os.path.join(input_directory, "Core", "Inc", "app_threadx.h"))

    if freertos_enable:
        system = "FreeRTOS"
    elif threadx_enable:
        system = "ThreadX"
    else:
        system = "None"

    with open(file_path, "w", encoding="utf-8", newline="\n") as f:
        f.write(LIBXR_CMAKE_TEMPLATE.replace("_LIBXR_SYSTEM_", system))
    logging.info(f"Generated LibXR.CMake at: {file_path}")
    logging.info("LibXR.CMake generated successfully.")

    main_cmake_path = os.path.join(input_directory, "CMakeLists.txt")
    if os.path.exists(main_cmake_path):
        cmake_content = read_text_with_fallback(main_cmake_path)

        if include_cmake_cmd not in cmake_content:
            with open(main_cmake_path, "a", encoding="utf-8", newline="\n") as f:
                f.write('\n# Add LibXR\n' + include_cmake_cmd)
            logging.info("LibXR.CMake included in CMakeLists.txt.")
        else:
            logging.info("LibXR.CMake already included in CMakeLists.txt.")
    else:
        logging.error("CMakeLists.txt not found.")
        exit(1)
