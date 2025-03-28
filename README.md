<h1 align="center">
<img src="https://github.com/Jiu-xiao/LibXR_CppCodeGenerator/raw/main/imgs/XRobot.jpeg" width="300">
</h1><br>

[![License](https://img.shields.io/badge/license-Apache--2.0-blue)](LICENSE)
[![GitHub Repo](https://img.shields.io/github/stars/Jiu-xiao/libxr?style=social)](https://github.com/Jiu-xiao/libxr)
[![Documentation](https://img.shields.io/badge/docs-online-brightgreen)](https://jiu-xiao.github.io/libxr/)
[![GitHub Issues](https://img.shields.io/github/issues/Jiu-xiao/LibXR_CppCodeGenerator)](https://github.com/Jiu-xiao/LibXR_CppCodeGenerator/issues)
[![CI/CD - Python Package](https://github.com/Jiu-xiao/LibXR_CppCodeGenerator/actions/workflows/python-publish.yml/badge.svg)](https://github.com/Jiu-xiao/LibXR_CppCodeGenerator/actions/workflows/python-publish.yml)

`libxr` is a Python package designed to automate embedded system development by parsing `.ioc` files and generating C++
code. It significantly reduces manual effort in STM32CubeMX-based projects.

## 🌟 Features

- **Automatic `.ioc` parsing**: Extracts GPIO, peripherals (UART, SPI, I2C, etc.), and DMA configurations.
- **C++ code generation**: Generates fully functional STM32-compatible code.
- **STM32CubeMX & FreeRTOS support**: Works seamlessly with STM32 and CMake build systems.
- **Automated Git management**: Initializes repositories and creates `.gitignore` files.
- **CMake automation**: Generates `CMakeLists.txt` for STM32 projects.

---

## 📥 Installation

### Install via `pip`

```sh
pip install libxr
```

### Install from source

```sh
git clone https://github.com/Jiu-xiao/LibXR_CppCodeGenerator.git
cd LibXR_CppCodeGenerator
pip install -e .
```

---

## 📌 API Reference

### `xr_cubemx_cfg`

This command automates STM32CubeMX project setup, including:

- Parsing `.ioc` files and generating YAML configuration.
- Generating C++ code.
- Modifying STM32 interrupt handlers.
- Creating `CMakeLists.txt` with Clang support (if enabled).
- Initializing Git and setting up `.gitignore`.
- Adding the `LibXR` submodule if missing.

**Usage:**

```sh
xr_cubemx_cfg [-h] -d DIRECTORY [-t TERMINAL] [-c]
```

**Required arguments:**

- `-d, --directory` (*str*): Path to the STM32CubeMX project.

**Optional arguments:**

- `-t, --terminal` (*str*): Specifies the terminal device source.
- `-c, --clang`: Enables Clang support for compilation.

---

### `xr_parse_ioc`

Parses the `.ioc` file in the specified directory and extracts peripheral configurations into a YAML file.

**Usage:**

```sh
xr_parse_ioc [-h] -d DIRECTORY [-o OUTPUT]
```

**Required arguments:**

- `-d, --directory` (*str*): Path to the STM32CubeMX project directory.

---

### `xr_gen_code`

Generates C++ code from the parsed `.ioc` YAML configuration and saves an additional YAML file for further
modifications.

**Usage:**

```sh
xr_gen_code [-h] -i INPUT [-o OUTPUT]
```

**Required arguments:**

- `-i, --input` (*str*): Path to the YAML file generated by `libxr_parse_ioc`.

**Outputs:**

- `app_main.cpp`: Generated STM32 C++ source file.
- `libxr_config.yaml`: Stores configuration options such as DMA buffer sizes, terminal settings, and peripheral
  mappings.

---

### `xr_stm32_it`

Modifies STM32 `_it.c` interrupt handler files to include necessary callback functions.

**Usage:**

```sh
xr_stm32_it [-h] input_dir
```

**Required arguments:**

- `input_dir` (*str*): Path to the STM32CubeMX project source directory.

---

### `xr_stm32_clang`

Generates `gcc-arm-none-eabi.cmake` for STM32 projects using Clang.

**Usage:**

```sh
xr_stm32_clang [-h] input_dir
```

**Required arguments:**

- `input_dir` (*str*): Path to the STM32CubeMX project.

---

### `xr_stm32_cmake`

Generates `LibXR.CMake` for STM32 projects using LibXR.

**Usage:**

```sh
xr_stm32_cmake [-h] input_dir
```

**Required arguments:**

- `input_dir` (*str*): Path to the STM32CubeMX project.

---

### .IOC file requirements

- **STM32CubeMX**: The `.ioc` file must be generated using STM32CubeMX.
- **CMake**: The project must be built using CMake.
- **DMA**: UART, SPI and I2C must have DMA enabled.

---

## After Generation

After the code generation is completed, users need to manually add the following include statement:

```cpp
#include "app_main.h"
```

Additionally, ensure that the function `app_main();` is called at an appropriate location:

- **For non-RTOS environments**: Call `app_main();` at the end of the `main` function.
- **For FreeRTOS environments**: Call `app_main();` at the beginning of the thread function.

---

## 🛠️ Contributing

We welcome community contributions! See our [contribution guidelines](CONTRIBUTING.md) for details.

Ways to contribute:

- 📝 **Submit pull requests** for new features or bug fixes.
- 🔍 **Review pull requests** from other contributors.
- 🐛 **Report issues** and suggest improvements.
- 📖 **Write documentation** and tutorials.
- 🎨 **Design graphics** for branding and promotions.

For detailed guidelines, visit our [contributing page](https://github.com/Jiu-xiao/libxr/blob/main/CONTRIBUTING.md).

---

## 📄 License

`libxr` is licensed under the **Apache-2.0** license. See the [LICENSE](LICENSE) file for more details.

---

## 🔗 Resources

- **Project Homepage**: [GitHub](https://github.com/Jiu-xiao/libxr)
- **Documentation**: [Online Docs](https://xrobot-org.github.io/)
- **Issue Tracker**: [Report Issues](https://github.com/Jiu-xiao/LibXR_CppCodeGenerator/issues)
- **Source Code**: [GitHub Repository](https://github.com/Jiu-xiao/LibXR_CppCodeGenerator)
