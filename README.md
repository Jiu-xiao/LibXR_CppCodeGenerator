<h1 align="center">
<img src="https://github.com/Jiu-xiao/LibXR_CppCodeGenerator/raw/main/imgs/XRobot.jpeg" width="300">
</h1><br>

[![License](https://img.shields.io/badge/license-Apache--2.0-blue)](LICENSE)
[![GitHub Repo](https://img.shields.io/github/stars/Jiu-xiao/libxr?style=social)](https://github.com/Jiu-xiao/libxr)
[![Documentation](https://img.shields.io/badge/docs-online-brightgreen)](https://jiu-xiao.github.io/libxr/)
[![GitHub Issues](https://img.shields.io/github/issues/Jiu-xiao/LibXR_CppCodeGenerator)](https://github.com/Jiu-xiao/LibXR_CppCodeGenerator/issues)
[![CI/CD - Python Package](https://github.com/Jiu-xiao/LibXR_CppCodeGenerator/actions/workflows/python-publish.yml/badge.svg)](https://github.com/Jiu-xiao/LibXR_CppCodeGenerator/actions/workflows/python-publish.yml)
[![FOSSA Status](https://app.fossa.com/api/projects/git%2Bgithub.com%2FJiu-xiao%2FLibXR_CppCodeGenerator.svg?type=shield)](https://app.fossa.com/projects/git%2Bgithub.com%2FJiu-xiao%2FLibXR_CppCodeGenerator?ref=badge_shield)

`libxr` 是一个 Python 包，用于自动化嵌入式系统开发。它通过解析硬件配置文件并生成对应的 C++ 工程代码，显著降低嵌入式开发中的重复性工作。目前默认支持 STM32 平台，后续将扩展至更多硬件体系结构。

`libxr` is a Python package for automating embedded system development. It parses hardware configuration files and generates corresponding C++ project code, significantly reducing repetitive manual work. STM32 is supported by default, with more hardware architectures planned.

## 🌟 Features 功能亮点

- 🧠 自动生成设备驱动和应用程序框架。  
  Automatically generates device drivers and application scaffolding.

- ⚙️ 支持多种后端架构，默认支持 STM32 平台。  
  Supports multiple backends; STM32 is the default.

- 🔌 支持多重别名注册与查找。  
  Supports multi-alias registration and lookup.

- 📦 可与 XRobot 框架集成，实现模块自动注册与调度管理。  
  Enables integration with the XRobot application framework.

## 📥 Installation 安装

### 使用pipx安装 (Install via `pipx`)

windows

```ps
python -m pip install --user pipx
python -m pipx ensurepath
pipx install libxr
pipx ensurepath
# Restart your terminal
```

linux

```bash
sudo apt install pipx
pipx install libxr
pipx ensurepath
# Restart your terminal
```

### 使用 pip 安装 (Install via `pip`)

```bash
pip install libxr
```

### 从源码安装 (Install from source)

```bash
git clone https://github.com/Jiu-xiao/LibXR_CppCodeGenerator.git
cd LibXR_CppCodeGenerator
python3 ./scripts/gen_libxr_version.py
pip install -e .
```

---

## 🔧 General 通用命令(跨平台支持)

以下命令适用于所有平台(如 STM32 及未来支持的架构)。  
These commands work across platforms (STM32 and others):

### `xr_parse`

```bash
xr_parse -i config.yaml
```

解析通用的 YAML 硬件配置文件，提取外设定义。  
Parses a generic YAML hardware configuration and extracts peripheral definitions.

### `xr_gen_code`

```bash
xr_gen_code -i config.yaml [--xrobot]
```

根据 YAML 配置生成平台无关的 C++ 硬件抽象层代码，可选生成 XRobot 集成代码。  
Generates platform-agnostic C++ hardware abstraction code from YAML.

---

## STM32 工程工具 (STM32 Project Tools)

### `xr_cubemx_cfg`

自动配置 STM32CubeMX 工程  
Automatically configures an STM32CubeMX project.

```bash
usage: xr_cubemx_cfg [-h] -d DIRECTORY [-t TERMINAL] [--xrobot]
```

解析 `.ioc` 文件，生成 YAML 和 C++ 驱动代码，补丁中断处理函数，并初始化项目结构  
Parses `.ioc`, generates YAML and C++ code, patches interrupt handlers, and initializes the project structure.

#### 🔧 必选参数 (Required)

- `-d, --directory <DIRECTORY>`：

  STM32CubeMX 工程路径  
  Path to the STM32CubeMX project.

#### ⚙️ 可选参数 (Optional)

- `-t, --terminal <TERMINAL>`：
  
  串口设备名称(如 `usart1` `usb`)  
  Terminal device name (e.g. `usart1` `usb`).

- `-c, --clang`：

  启用 Clang 构建支持 (此选项已经弃用) 
  Enable Clang build support. (This option has been deprecated) 

- `--xrobot`：

  生成 XRobot Glue 代码  
  Enable XRobot glue code generation.

- `--commit`
  
  指定 LibXR 仓库commit版本  
  Specify the LibXR repository commit version

#### 📦 输出内容 (Outputs)

- `.config.yaml`:

  自动生成的 C++ 驱动代码(如 `app_main.cpp`  
  Generated C++ driver code (e.g. `app_main.cpp`)

- 补丁后的中断处理函数(如 `stm32xx_it.c`)  
  Patched interrupt handlers (e.g. `stm32xx_it.c`)

- `CMakeLists.txt`、`.gitignore`  

- 初始化的 Git 仓库及 LibXR 子模块  
  Initialized Git repository and LibXR submodule

---

### `xr_parse_ioc`

自动解析 STM32CubeMX 工程配置  
Parses `.ioc` files from STM32CubeMX projects and exports structured YAML.

```bash
usage: xr_parse_ioc [-h] -d DIRECTORY [-o OUTPUT] [--verbose]
```

解析 `.ioc` 文件为 `.config.yaml`，并在终端输出解析摘要  
Parses `.ioc` files and creates `.config.yaml` with a readable summary.

#### 🔧 必选参数 (Required)

- `-d, --directory <DIRECTORY>`  
  `.ioc` 文件所在目录路径  
  Path to the input directory containing `.ioc` files.

#### ⚙️ 可选参数 (Optional)

- `-o, --output <FILE>`  
  自定义 YAML 输出路径(默认与 `.ioc` 同名)  
  Custom YAML output path (default: `<input_file>.yaml`).

- `--verbose`  
  启用调试日志，输出详细解析过程  
  Enable verbose logging.

#### 📦 输出内容 (Outputs)

- `.config.yaml`：

  包含 GPIO、外设、DMA、FreeRTOS、MCU 等配置  
  YAML file containing GPIO, peripheral, DMA, FreeRTOS, and MCU configurations.

- 控制台摘要：MCU 信息、GPIO 数量、外设统计等  
  Console summary: MCU information, GPIO count, peripheral statistics, etc.

---

### `xr_gen_code_stm32`

根据 YAML 配置生成 STM32 硬件抽象层代码，可选生成 XRobot 集成代码。  
Generates STM32 application code from YAML.

```bash
usage: xr_gen_code_stm32 [-h] -i INPUT -o OUTPUT [--xrobot] [--hw-cntr] [--libxr-config LIBXR_CONFIG]
```

#### 🔧 Required

- `-i`：

  `.config.yaml` 配置文件路径  
  Path to `.config.yaml`

- `-o`：

  生成代码输出目录  
  Output directory

#### ⚙️ Optional

- `--xrobot`：

  启用 XRobot glue 代码生成  
  Enable XRobot glue generation

- `--hw-cntr`  
  生成 LibXR HardwareContainer 定义及 app_framework.hpp 头文件（可用于非 XRobot 项目）  
  Generate LibXR HardwareContainer definition and include app_framework.hpp header (can be used without XRobot)

- `--libxr-config`：

  自定义 libxr_config.yaml 路径(可为本地或远程)  
  Path or URL to runtime config YAML

#### 📦 Outputs

- `app_main.cpp`：  
  主入口文件，包含所有初始化逻辑  
  Main entry point with all initialization logic

- `libxr_config.yaml`：  
  运行时配置文件，可自定义缓冲区大小、队列等参数  
  Runtime config YAML, can be customized with buffer size, queue, etc.

- `flash_map.hpp`：  
  自动生成的 Flash 扇区表，供 Flash 抽象层使用  
  Auto-generated flash sector layout for use with Flash abstraction layer

---

### `xr_stm32_flash`

解析 STM32 型号，生成 Flash 扇区信息表（YAML 格式输出）。  
Parses STM32 model name and generates flash layout info (YAML output).

```bash
usage: xr_stm32_flash <STM32_MODEL>
```

### 🧠 功能说明 (Functionality)

- 根据 STM32 型号名称自动推导 Flash 大小  
  Automatically infers flash size from the STM32 model string

- 根据芯片系列（如 F1/F4/H7/U5 等）生成对应的扇区布局  
  Generates sector layout depending on the chip series (e.g., F1/F4/H7/U5)

- 输出包括每个扇区的地址、大小和索引  
  Output includes address, size, and index of each sector

### 📦 输出内容 (Outputs)

- YAML 格式的 Flash 信息  
  Flash info in YAML format:

```yaml
model: STM32F103C8
flash_base: '0x08000000'
flash_size_kb: 64
sectors:
- index: 0
  address: '0x08000000'
  size_kb: 1.0
- index: 1
  address: '0x08000400'
  size_kb: 1.0
  ...
```

---

### `xr_stm32_it`

批量修改 STM32 中断处理文件，插入 UART IDLE 回调和 USB F1 回调支持。
Modifies STM32 interrupt handlers to add UART IDLE callback and USB F1 callback support for LibXR.

```bash
usage: xr_stm32_it [-h] input_dir
```

#### 🔧 必选参数 (Required)

- `input_dir`：

  包含 `*_it.c` 文件的文件夹路径  
  Directory containing the `*_it.c` source files.

#### ⚙️ 功能说明 (Functionality)

- 查找每个 `*_it.c` 文件中的 `HAL_UART_IRQHandler(&huartX)` 调用  
  Find `HAL_UART_IRQHandler(&huartX)` calls in each `*_it.c` file

- 查找 `USB_HP_CAN1_TX_IRQHandler` 和 `USB_LP_CAN1_RX0_IRQHandler` 中断函数  
  Find `USB_HP_CAN1_TX_IRQHandler` and `USB_LP_CAN1_RX0_IRQHandler` handlers

- 向对应中断函数的 `/* USER CODE BEGIN XXX_IRQn 0/1 */` 区域插入：  
  Add to the `/* USER CODE BEGIN XXX_IRQn 0/1 */` section of the corresponding interrupt function
  
```c
  /* LibXR UART IDLE callback (Auto-generated) */
#ifdef HAL_UART_MODULE_ENABLED
  STM32_UART_ISR_Handler_IDLE(&huartX);
#endif
```

- 对于 USB 中断（仅 STM32F1，且启用 HAL_PCD_MODULE），插入：  
  For USB interrupts (STM32F1 only, with HAL_PCD_MODULE enabled)

```c
/* LibXR USB Tx Cplt callback (Auto-generated, For STM32F1) */
#if defined(STM32F1) && defined(HAL_PCD_MODULE_ENABLED)
  STM32_USB_ISR_Handler_F1();
#endif
```

- 若未定义 `STM32_UART_ISR_Handler_IDLE`和 `STM32_USB_ISR_Handler_F1`的 `extern` 声明，将插入至 `/* USER CODE BEGIN 0 */` 区域下  
  Insert at `/* USER CODE BEGIN 0 */` if `STM32_UART_ISR_Handler_IDLE` and `STM32_USB_ISR_Handler_F1` is not defined

- 支持多个 UART 和 USB 接口  
  Support for multiple UART and USB interfaces

#### 📦 输出内容 (Outputs)

- 被修改的 `*_it.c` 文件  
  Modified `*_it.c` files

- 控制台输出修改摘要，包括修改的文件与函数名称  
  Console output summary, including modified files and function names

---

### `xr_stm32_clang`

自动修改 STM32 工程，适配 Clang 编译工具链。  
Creates Clang-compatible toolchain file.

```bash
usage: xr_stm32_clang [-h] input_dir
```

#### 工具已弃用 / Deprecated Notice

自 STM32CubeMX 15.0 起，官方已原生支持 Clang 工具链，并会自动生成 starm-clang.cmake 文件。

Since STM32CubeMX 15.0, native Clang support is provided and starm-clang.cmake will be generated automatically.

#### 🔧 必选参数 (Required)

- `input_dir`：

  包含 `gcc-arm-none-eabi.cmake` 的 STM32 工程目录路径  
  Directory containing `gcc-arm-none-eabi.cmake` for the STM32 project

#### ⚙️ 功能说明 (Functionality)

- 在 `{input_dir}/cmake/` 中查找 `gcc-arm-none-eabi.cmake` 文件  
  Locate `gcc-arm-none-eabi.cmake` inside `{input_dir}/cmake/`

- 解析其中的 `linker_script` 路径与 `-mcpu` 编译器参数  
  Extract `linker_script` path and `-mcpu` target flag

- 自动生成 `gcc-arm-none-eabi.cmake` 文件，适配 Clang 编译工具链  
  Generate Clang-compatible `gcc-arm-none-eabi.cmake` toolchain file

- 自动修改 `CMakeLists.txt` 文件，插入  
  Automatically modify `CMakeLists.txt` to include:
  
  ```cmake
  include("cmake/gcc-arm-none-eabi.cmake")
  ```

#### 📦 输出内容 (Outputs)

- 覆盖原有 `gcc-arm-none-eabi.cmake` 文件  
  Overwrites original `gcc-arm-none-eabi.cmake` file

- 自动添加或更新 `CMakeLists.txt` 文件中的 include 指令  
  Auto-updates `CMakeLists.txt` to include the toolchain file

---

### `xr_libxr_cmake`

为 STM32CubeMX 工程生成 `LibXR.CMake` 配置，并自动集成至 `CMakeLists.txt`。  
Generates `LibXR.CMake` file and injects it into the STM32CubeMX CMake project.

```bash
usage: xr_libxr_cmake [-h] input_dir
```

#### 🔧 必选参数 (Required)

- `input_dir`：

  指定 CubeMX 生成的 CMake 工程根目录  
  Path to the CubeMX-generated CMake project root

#### ⚙️ 功能说明 (Functionality)

- 自动生成 `cmake/LibXR.CMake` 文件，内容包括：  
  Generate `cmake/LibXR.CMake` containing:
  
  - 添加 `LibXR` 子目录  
    Add `LibXR` as a subdirectory
  
  - 链接 `xr` 静态库  
    Link the `xr` static library
  
  - 添加 `Core/Inc`、`User` 目录为包含路径  
    Include `Core/Inc` and `User` directories
  
  - 添加 `User/*.cpp` 为源文件  
    Add `User/*.cpp` to project sources

- 自动检测是否启用 FreeRTOS：  
  Auto-detect FreeRTOS configuration:
  
  - 存在 `Core/Inc/FreeRTOSConfig.h` → `LIBXR_SYSTEM=FreeRTOS`
  - 否则设置为 `None`

- 自动删除旧的 `build/` 目录(如存在)  
  Automatically deletes existing `build/` directory if found

- 自动向主 `CMakeLists.txt` 添加以下指令(若尚未包含)：  
  Auto-appends the following line to `CMakeLists.txt` if missing:

  ```cmake
  include(${CMAKE_CURRENT_LIST_DIR}/cmake/LibXR.CMake)
  ```

#### 📦 输出内容 (Outputs)

- 生成 `cmake/LibXR.CMake` 文件  
  Generates `cmake/LibXR.CMake` file

- 修改主工程的 `CMakeLists.txt`，插入 `include(...)`  
  Updates `CMakeLists.txt` to include `LibXR.CMake`

- 删除原有构建缓存目录 `build/`(如存在)  
  Deletes the old `build/` directory if present

---

### STM32 工程要求  (STM32 Project Requirements)

#### 📁 项目结构要求(Project Structure)

- 必须为 **STM32CubeMX 导出的 CMake 工程**  
  Must be a CMake project exported from STM32CubeMX

- 项目应包含以下路径：  
  Project should contain the following directories:

  - `xx.ioc`
  - `CMakeLists.txt`
  - `cmake/gcc-arm-none-eabi.cmake`
  - `Core/Inc`, `Core/Src`

#### ⚙️ 配置要求(Peripheral & Middleware)

- 所有 **UART / SPI / I2C** 外设必须启用 **DMA**  
  All **UART / SPI / I2C** peripherals must have **DMA** enabled

- 如果ADC启用了DMA，请开启连续转换模式  
  If ADC has DMA enabled, enable continuous mode

- 推荐启用 **FreeRTOS**，自动生成 `FreeRTOSConfig.h`  
  Recommended to enable **FreeRTOS** and generate `FreeRTOSConfig.h`

- 若使用 USB 作为终端：  
  If using USB as a terminal:

  - 启用 `USB_DEVICE` 中间件，并选择 `Communication Device Class (CDC)`  
    Enable `USB_DEVICE` middleware and select `Communication Device Class (CDC)`

#### ⏱️ Timebase 配置建议(Timebase Configuration)

> ✅ 强烈推荐使用 `TIM6`/`TIM7` 等 Timer 作为 Timebase  
    Strongly recommended to use `TIM6`/`TIM7` Timers as Timebase  
> ✅ 并将该中断优先级设置为 **最高(0)**  
    And set the interrupt priority to **highest (0)**

---

## 🧩 代码生成后操作 (After Code Generation)

生成代码后，你需要**手动添加**以下内容：  
After generating code, you must **manually add** the following:  

```cpp
#include "app_main.h"
```

并在合适位置调用 `app_main();`：  
And call `app_main();` in the appropriate location:

| 场景 (Scenario)       | 添加位置        |Where to add|
|-----------------------|------------------------------------| -----------|
| 🟢 Bare metal 裸机工程 | `main()` 函数末尾   | End of `main()` |
| 🔵 FreeRTOS 工程       | 线程入口       | Thread entry function |

---

## LibXR / LibXR_CppCodeGenerator / XRobot Relationship

LibXR、LibXR_CppCodeGenerator 与 XRobot 三者形成了一套完整的嵌入式与机器人软件开发体系，分工明确，协同紧密。  
LibXR, LibXR_CppCodeGenerator and XRobot together form a complete software ecosystem for embedded and robotics development, with clear separation of concerns and tight integration.

---

### 🧠 LibXR

**LibXR 是跨平台的驱动抽象与工具库**，支持 STM32、Linux 等平台，包含：  
LibXR is a cross-platform driver abstraction and utility library supporting STM32, Linux, and more. It provides:

- 通用外设接口封装  
  Unified peripheral interface abstraction  
- 嵌入式组件（如 Terminal、PowerManager、Database 等）  
  Embedded modules like Terminal, PowerManager, Database, etc.  
- FreeRTOS / bare-metal 支持  
  FreeRTOS and bare-metal support  
- 机器人运动学与导航  
  Kinematics and navigation libraries for robotics  
- 自动代码生成支持  
  Code generation support

#### 🔗 Links

- **Repository**: [libxr](https://github.com/Jiu-xiao/libxr)  
- **API Documentation**: [API](https://jiu-xiao.github.io/libxr/)  
- **Issues**: [Issue Tracker](https://github.com/Jiu-xiao/libxr/issues)

---

### 🔧 LibXR_CppCodeGenerator

**LibXR_CppCodeGenerator 是用于 LibXR 的代码生成工具链**，当前支持 STM32 + CubeMX，未来将扩展至 Zephyr、ESP-IDF 等平台。  
LibXR_CppCodeGenerator is a code generation toolchain for LibXR. It currently supports STM32 with CubeMX, and is planned to support Zephyr, ESP-IDF, and more.

- 从不同平台的工程文件生成 `.yaml` 配置  
  Parse project files from different platforms to generate `.yaml` configurations
- 基于 `.yaml` 自动生成 `app_main.cpp`、中断、CMake 等  
  Generate `app_main.cpp`, interrupt handlers, and CMake integration  
- 支持 `XRobot` glue 层集成  
  Supports optional integration with XRobot framework  
- 支持用户代码保留与多文件结构  
  Preserves user code blocks and supports modular output

#### 🔗 Links

- **Repository**: [LibXR_CppCodeGenerator](https://github.com/Jiu-xiao/LibXR_CppCodeGenerator)  
- **Documentation and Releases**: [PyPI](https://pypi.org/project/libxr/)  
- **Issues**: [Issue Tracker](https://github.com/Jiu-xiao/LibXR_CppCodeGenerator/issues)

---

### 🤖 XRobot

XRobot 是一个轻量级的模块化应用管理框架，专为嵌入式设备而设计。它本身不包含任何驱动或业务代码，专注于模块的注册、调度、生命周期管理、事件处理与参数配置。  
**XRobot is a lightweight modular application management framework designed for embedded systems.**  
It does not include any drivers or business logic by itself. Instead, it focuses on module registration, scheduling, lifecycle management, event handling, and parameter configuration.

- 模块注册与生命周期管理  
  Module registration and lifecycle management  
- 参数管理 / 配置系统 / 事件系统  
  Parameter management, configuration system, and event system  
- ApplicationRunner / ThreadManager 等应用调度器  
  ApplicationRunner and ThreadManager for runtime coordination  
- 不直接访问硬件，依赖 LibXR 的 PeripheralManager  
  Does not access hardware directly, relies on LibXR's PeripheralManager

---

#### ✅ Recommended For 推荐使用场景

- 拥有多个子模块（如传感器、通信、控制器）且希望统一管理初始化、调度与资源依赖  
  For projects with multiple submodules (e.g., sensors, communication, controllers) needing unified lifecycle and dependency management.

- 希望构建平台无关的应用层逻辑，与底层驱动解耦  
  For building platform-independent application logic decoupled from hardware drivers.

- 与 **LibXR** 结合使用，实现自动注册硬件对象（通过 `HardwareContainer`）  
  When used with **LibXR**, supports automatic hardware registration via `HardwareContainer`.

- 支持生成模块入口代码、配置逻辑名与硬件名的映射，便于快速适配不同硬件配置  
  Supports generating module entry code and logical-to-physical hardware name mapping for quick adaptation to different platforms.

#### 🔗 Links

- **Repository**: [XRobot](https://github.com/xrobot-org/XRobot)  
- **Documentation**: [GitHub Pages](https://xrobot-org.github.io)  
- **Releases**: [PyPI](https://pypi.org/project/xrobot)  
- **Issues**: [Issue Tracker](https://github.com/xrobot-org/XRobot/issues)

---

## 📄 License

Licensed under **Apache-2.0**. See [LICENSE](LICENSE).


[![FOSSA Status](https://app.fossa.com/api/projects/git%2Bgithub.com%2FJiu-xiao%2FLibXR_CppCodeGenerator.svg?type=large)](https://app.fossa.com/projects/git%2Bgithub.com%2FJiu-xiao%2FLibXR_CppCodeGenerator?ref=badge_large)