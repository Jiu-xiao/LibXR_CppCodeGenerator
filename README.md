
<h1 align="center">
<img src="https://github.com/Jiu-xiao/LibXR_CppCodeGenerator/raw/main/imgs/XRobot.jpeg" width="300">
</h1><br>

[![License](https://img.shields.io/badge/license-Apache--2.0-blue)](LICENSE)
[![GitHub Repo](https://img.shields.io/github/stars/Jiu-xiao/libxr?style=social)](https://github.com/Jiu-xiao/libxr)
[![Documentation](https://img.shields.io/badge/docs-online-brightgreen)](https://jiu-xiao.github.io/libxr/)
[![GitHub Issues](https://img.shields.io/github/issues/Jiu-xiao/LibXR_CppCodeGenerator)](https://github.com/Jiu-xiao/LibXR_CppCodeGenerator/issues)
[![CI/CD - Python Package](https://github.com/Jiu-xiao/LibXR_CppCodeGenerator/actions/workflows/python-publish.yml/badge.svg)](https://github.com/Jiu-xiao/LibXR_CppCodeGenerator/actions/workflows/python-publish.yml)

`libxr` is a Python package that automates embedded system development by parsing `.ioc` files and generating C++ code. It significantly reduces manual effort in STM32CubeMX-based projects.

## 🌟 Features

- 🧠 **Hardware-Aware Codegen**: Automatically generates device drivers and application scaffolding.
- ⚙️ **Modular Architecture**: Supports multiple backends; STM32 is the default.
- 🔌 **Peripheral Aliasing**: Supports multi-alias registration and lookup.
- 📦 **Optional XRobot2.0 Glue**: Enables integration with the XRobot application framework.

## 📥 Installation

### Install via `pip`

```bash
pip install libxr
```

### Install from source

```bash
git clone https://github.com/Jiu-xiao/LibXR_CppCodeGenerator.git
cd LibXR_CppCodeGenerator
pip install -e .
```

---

## 🔧 General (Cross-Platform)

These commands work across platforms (STM32 and others):

### `xr_parse`

```bash
xr_parse -i config.yaml
```

Parses a generic YAML hardware configuration and extracts peripheral definitions.

### `xr_gen_code`

```bash
xr_gen_code -i config.yaml [--xrobot]
```

Generates platform-agnostic C++ hardware abstraction code from YAML.

---

## STM32 Project Tools

### `xr_cubemx_cfg`

```bash
usage: xr_cubemx_cfg [-h] -d DIRECTORY [-t TERMINAL] [-c] [--xrobot]
```

Automatically configures an STM32CubeMX project by parsing `.ioc`, generating YAML and C++ code, modifying interrupt handlers, and initializing project files.

#### Required

- `-d, --directory <DIRECTORY>`: Path to the STM32CubeMX project.

#### Optional

- `-t, --terminal <TERMINAL>`: Terminal device name.
- `-c, --clang`: Enable Clang support.
- `--xrobot`: Enable XRobot glue code.

#### Outputs

- `.config.yaml`, C++ drivers, `app_main.cpp`
- Modified interrupt handlers
- `CMakeLists.txt`, `.gitignore`
- Initialized Git repo with LibXR submodule

---

### `xr_parse_ioc`

```bash
usage: xr_parse_ioc [-h] -d DIRECTORY
```

Parses `.ioc` and creates a `.config.yaml`.

---

### `xr_gen_code_stm32`

```bash
usage: xr_gen_code_stm32 [-h] -i INPUT -o OUTPUT [--xrobot] [--libxr-config LIBXR_CONFIG]
```

Generates STM32 application code from YAML.

#### Required

- `-i`: Path to `.config.yaml`
- `-o`: Output directory

#### Optional

- `--xrobot`: Enable XRobot glue
- `--libxr-config`: Path to runtime config YAML

#### Outputs

- `app_main.cpp`, `libxr_config.yaml`

---

### `xr_stm32_it`

```bash
usage: xr_stm32_it [-h] input_dir
```

Modifies STM32 interrupt handlers to support LibXR.

---

### `xr_stm32_clang`

```bash
usage: xr_stm32_clang [-h] input_dir
```

Creates Clang-compatible toolchain file.

---

### `xr_stm32_cmake`

```bash
usage: xr_stm32_cmake [-h] input_dir
```

Creates `LibXR.CMake` for CMake builds.

---

## .IOC Requirements

- Must be from STM32CubeMX
- CMake-based project
- DMA must be enabled for UART/SPI/I2C

---

## After Code Generation

You must manually add:

```cpp
#include "app_main.h"
```

And call `app_main();` appropriately:

- **Bare metal**: at the end of `main()`
- **FreeRTOS**: inside the thread entry

---

## 🛠️ Contributing

We welcome your contributions! See [CONTRIBUTING.md](CONTRIBUTING.md).

You can:

- 📝 Submit pull requests
- 🔍 Review others' code
- 🐛 Report bugs
- 📖 Write docs
- 🎨 Design assets

---

## 📄 License

Licensed under **Apache-2.0**. See [LICENSE](LICENSE).

---

## 🔗 Resources

- [GitHub Homepage](https://github.com/Jiu-xiao/libxr)
- [Online Docs](https://xrobot-org.github.io/)
- [Issue Tracker](https://github.com/Jiu-xiao/LibXR_CppCodeGenerator/issues)
- [Source Code](https://github.com/Jiu-xiao/LibXR_CppCodeGenerator)
