<h1 align="center">
<img src="imgs/XRobot.jpeg" width="300">
</h1><br>

[![License](https://img.shields.io/badge/license-Apache--2.0-blue)](LICENSE)
[![GitHub Repo](https://img.shields.io/github/stars/Jiu-xiao/LibXR_CppCodeGenerator?style=social)](https://github.com/Jiu-xiao/libxr)
[![Documentation](https://img.shields.io/badge/docs-online-brightgreen)](https://xrobot-org.github.io/)
[![GitHub Issues](https://img.shields.io/github/issues/Jiu-xiao/LibXR_CppCodeGenerator)](https://github.com/Jiu-xiao/LibXR_CppCodeGenerator/issues)
[![Contributors](https://img.shields.io/github/contributors/Jiu-xiao/libxr)](https://github.com/Jiu-xiao/libxr/graphs/contributors)

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

### `libxr_config_cubemx_project`

This command **integrates all other commands** into a single automated process, including:

- Parsing `.ioc` files.
- Generating C++ code.
- Modifying STM32 interrupt handlers.
- Creating `CMakeLists.txt`.
- Initializing Git and setting up the project structure.

**Usage:**

```sh
libxr_config_cubemx_project [-h] -d DIRECTORY [-t TERMINAL]
```

**Required arguments:**

- `-d, --directory` (*str*): Path to the STM32CubeMX project.
- `-t, --terminal` (*str*, optional): Optional terminal device source.

---

### `libxr_parse_ioc`

Parses the `.ioc` file in the specified directory and extracts peripheral configurations into a JSON file.

**Usage:**

```sh
libxr_parse_ioc [-h] -d DIRECTORY [-o OUTPUT]
```

**Required arguments:**

- `-d, --directory` (*str*): Path to the STM32CubeMX project directory.

---

### `libxr_generate_code`

Generates C++ code from the parsed `.ioc` JSON configuration and saves an additional JSON file for further
modifications.

**Usage:**

```sh
libxr_generate_code [-h] -i INPUT [-o OUTPUT]
```

**Required arguments:**

- `-i, --input` (*str*): Path to the JSON file generated by `libxr_parse_ioc`.

**Outputs:**

- `app_main.cpp`: Generated STM32 C++ source file.
- `libxr_config.json`: Stores configuration options such as DMA buffer sizes, terminal settings, and peripheral
  mappings.

---

### `libxr_generate_stm32_it`

Modifies STM32 `_it.c` interrupt handler files to include necessary callback functions.

**Usage:**

```sh
libxr_generate_stm32_it [-h] input_dir
```

**Required arguments:**

- `input_dir` (*str*): Path to the STM32CubeMX project source directory.

---

### `libxr_generate_stm32_cmake`

Generates `CMakeLists.txt` for STM32 projects.

**Usage:**

```sh
libxr_generate_stm32_cmake [-h] input_dir
```

**Required arguments:**

- `input_dir` (*str*): Path to the STM32CubeMX project.

---

### .IOC file requirements

* **STM32CubeMX**: The `.ioc` file must be generated using STM32CubeMX.
* **CMake**: The project must be built using CMake.
* **DMA**: UART, SPI and I2C must have DMA enabled.

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