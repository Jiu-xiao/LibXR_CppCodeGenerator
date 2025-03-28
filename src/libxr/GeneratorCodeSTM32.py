#!/usr/bin/env python
"""STM32 Peripheral Code Generator - Core Module (Optimized)"""

import argparse
import logging
import os
import re
import sys

import yaml

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")

# --------------------------
# Global Configuration
# --------------------------
device_aliases = {"power_manager": {"type": "PowerManager", "aliases": ["power_manager"]}}
libxr_settings = {
    "terminal_source": "",
    "software_timer": {"priority": 2, "stack_depth": 512},
    "SPI": {},
    "I2C": {},
    "USART": {},
    "ADC": {},
    "TIM": {},
    "CAN": {},
    "FDCAN": {},
    "USB": {},
    "Terminal": {
        "READ_BUFF_SIZE": 32,
        "MAX_LINE_SIZE": 32,
        "MAX_ARG_NUMBER": 5,
        "MAX_HISTORY_NUMBER": 5
    },
    "SYSTEM": "None"
}


# --------------------------
# Configuration Initialization
# --------------------------
def initialize_device_aliases(use_xrobot: bool) -> None:
    global device_aliases
    device_aliases.clear()

    if not use_xrobot:
        return

    saved_aliases = libxr_settings.get("device_aliases", {})

    # 插入默认设备
    if "power_manager" not in saved_aliases:
        saved_aliases["power_manager"] = {
            "type": "PowerManager",
            "aliases": ["power_manager"]
        }

    device_aliases.update(saved_aliases)


# --------------------------
# CLI Arguments
# --------------------------
def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Generate STM32 Peripheral Initialization Code",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument("-i", "--input", required=True,
                        help="Input YAML configuration file path")
    parser.add_argument("-o", "--output", required=True,
                        help="Output C++ file path")
    parser.add_argument("--xrobot", action="store_true",
                        help="Enable XRobot framework integration")
    return parser.parse_args()


# --------------------------
# Device Registration
# --------------------------
def _register_device(name: str, dev_type: str):
    global device_aliases
    if name not in device_aliases:
        device_aliases[name] = {
            "type": dev_type,
            "aliases": [name]
        }


# --------------------------
# Peripheral Instance Generation
# --------------------------
def generate_peripheral_instances(project_data: dict) -> str:
    """Generate initialization code for all peripherals with topological sorting."""
    code_sections = {
        "adc": [],
        "pwm": [],
        "main": []
    }

    for p_type, instances in project_data.get("Peripherals", {}).items():
        for instance_name, config in instances.items():
            section, code = PeripheralFactory.create(p_type, instance_name, config)
            if section in code_sections:
                code_sections[section].append(code)

    # Assemble code in correct order: ADC config -> PWM -> Main peripherals
    return "\n".join([
        "\n".join(code_sections["adc"]),
        "\n".join(code_sections["pwm"]),
        "\n".join(code_sections["main"])
    ])


# --------------------------
# Configuration Loading
# --------------------------
def load_configuration(file_path: str, use_xrobot: bool) -> dict:
    """Load and validate project YAML configuration with enhanced error reporting."""
    try:
        with open(file_path, "r", encoding="utf-8") as f:
            config = yaml.safe_load(f)

            if use_xrobot:
                if 'device_aliases' in config:
                    new_aliases = {}
                    for dev, entry in config['device_aliases'].items():
                        if isinstance(entry, list):
                            new_aliases[dev] = {
                                "type": "Unknown",
                                "aliases": entry
                            }
                        elif isinstance(entry, str):
                            new_aliases[dev] = {
                                "type": "Unknown",
                                "aliases": [entry]
                            }
                        elif isinstance(entry, dict):
                            # 兼容新版格式（已带 type 和 aliases）
                            new_aliases[dev] = {
                                "type": entry.get("type", "Unknown"),
                                "aliases": entry.get("aliases", [])
                                if isinstance(entry.get("aliases"), list)
                                else [entry.get("aliases")]
                            }
                    libxr_settings['device_aliases'] = new_aliases

            # Basic schema validation
            required_sections = ["Mcu", "GPIO", "Peripherals"]
            for section in required_sections:
                if section not in config:
                    raise ValueError(f"Missing required section: {section}")

            # Detect RTOS
            if 'FreeRTOS' in config:
                libxr_settings['SYSTEM'] = 'FreeRTOS'
                logging.info("Detected FreeRTOS configuration")
            elif 'ThreadX' in config:
                libxr_settings['SYSTEM'] = 'ThreadX'
            else:
                libxr_settings['SYSTEM'] = 'None'

            # Software timer config
            if 'software_timer' in config:
                libxr_settings['software_timer'].update(config['software_timer'])

            # Terminal source
            if 'terminal_source' in config:
                libxr_settings['terminal_source'] = config['terminal_source']

            return config
    except FileNotFoundError:
        logging.error(f"Configuration file not found: {file_path}")
        sys.exit(1)
    except yaml.YAMLError as e:
        logging.error(f"YAML syntax error: {str(e)}")
        sys.exit(1)
    except ValueError as e:
        logging.error(f"Configuration validation failed: {str(e)}")
        sys.exit(1)


# --------------------------
# Library Configuration
# --------------------------
def load_libxr_config(output_dir: str) -> None:
    """Load or create library configuration with version compatibility check."""
    global libxr_settings
    config_path = os.path.join(output_dir, "libxr_config.yaml")

    try:
        if os.path.exists(config_path):
            with open(config_path, "r", encoding="utf-8") as f:
                saved_config = yaml.safe_load(f) or {}

                # Version compatibility check
                if saved_config.get("config_version", 1) > 1:
                    logging.warning("Config file format is newer than expected")

                # Merge configurations
                libxr_settings = _deep_merge(libxr_settings, saved_config)
        else:
            logging.info("Creating new library configuration file")
            os.makedirs(output_dir, exist_ok=True)
            with open(config_path, "w", encoding="utf-8") as f:
                yaml.dump(libxr_settings, f, allow_unicode=True, sort_keys=False)

    except Exception as e:
        logging.warning(f"Failed to process library config: {str(e)}")


def _deep_merge(base: dict, update: dict) -> dict:
    """Recursively merge nested dictionaries with type checking."""
    for key, value in update.items():
        if isinstance(value, dict):
            node = base.setdefault(key, {})
            if isinstance(node, dict):
                _deep_merge(node, value)
            else:
                logging.warning(f"Config type conflict for key '{key}', expected dict")
        else:
            base[key] = value
    return base


# --------------------------
# GPIO Configuration
# --------------------------
def _sanitize_cpp_identifier(name: str) -> str:
    return re.sub(r'\W|^(?=\d)', '_', name)

def generate_gpio_alias(port: str, gpio_data: dict, project_data: dict) -> str:
    base_port = port.split("-")[0]
    port_define = f"GPIO{base_port[1]}"
    pin_num = int(base_port[2:])
    pin_define = f"GPIO_PIN_{pin_num}"
    label = gpio_data.get("Label", "")

    if label:
        port_define = f"{label}_GPIO_Port"
        pin_define = f"{label}_Pin"

    irq_define = _get_exti_irq(pin_num, base_port, gpio_data.get("GPXTI", False),
                               project_data.get("Mcu", {}).get("Family", "STM32F4"))
    irq_str = f", {irq_define}" if irq_define else ""

    var_name = _sanitize_cpp_identifier(label or port)

    _register_device(var_name, "GPIO")

    return f"{var_name}({port_define}, {pin_define}{irq_str})"


def _get_exti_irq(pin_num: int, port: str, is_exti: bool, mcu_family: str) -> str:
    if not is_exti:
        return ""

    if mcu_family.startswith("STM32WB0"):
        if port.startswith("PA"):
            return "GPIOA_IRQn"
        elif port.startswith("PB"):
            return "GPIOB_IRQn"

    if mcu_family == "STM32F0" or mcu_family == 'STM32G0' or mcu_family == 'STM32L0':
        if pin_num <= 1: return "EXTI0_1_IRQn"
        if pin_num <= 3: return "EXTI2_3_IRQn"
        return "EXTI4_15_IRQn"
    else:
        if 5 <= pin_num <= 9: return "EXTI9_5_IRQn"
        if 10 <= pin_num <= 15: return "EXTI15_10_IRQn"
        return f"EXTI{pin_num}_IRQn"


# --------------------------
# DMA Configuration
# --------------------------
DMA_DEFAULT_SIZES = {
    "SPI": {"tx": 32, "rx": 32},
    "USART": {"tx": 128, "rx": 128},
    "I2C": {"buffer": 32},
    "ADC": {"buffer": 128}
}


def generate_dma_resources(project_data: dict) -> str:
    """Generate DMA buffers for all peripherals with enhanced compatibility"""
    dma_code = []

    for p_type_raw, instances in project_data.get("Peripherals", {}).items():
        match = re.match(r'([A-Za-z0-9]+?)(\d*)$', p_type_raw)
        p_type_base = match.group(1).upper() if match else p_type_raw.upper()

        if p_type_base not in libxr_settings:
            libxr_settings[p_type_base] = {}

        if p_type_base in ["SPI", "USART", "UART", "LPUART"]:
            for instance, config in instances.items():
                dma_tx = config.get("DMA_TX", "DISABLE") == "ENABLE"
                dma_rx = config.get("DMA_RX", "DISABLE") == "ENABLE"
                if not (dma_tx or dma_rx):
                    continue

                instance_lower = instance.lower()
                instance_config = libxr_settings[p_type_base].setdefault(instance_lower, {})

                tx_size = instance_config.setdefault(
                    "tx_buffer_size",
                    DMA_DEFAULT_SIZES[p_type_base]["tx"]
                )
                rx_size = instance_config.setdefault(
                    "rx_buffer_size",
                    DMA_DEFAULT_SIZES[p_type_base]["rx"]
                )

                buf_code = []
                if dma_tx:
                    buf_code.append(f"static uint8_t {instance_lower}_tx_buf[{tx_size}];")
                if dma_rx:
                    buf_code.append(f"static uint8_t {instance_lower}_rx_buf[{rx_size}];")

                if buf_code:
                    dma_code.append("\n".join(buf_code))

        elif p_type_base in ["I2C", "ADC"]:
            for instance, config in instances.items():
                instance_lower = instance.lower()
                instance_config = libxr_settings[p_type_base].setdefault(instance_lower, {})

                buf_size = instance_config.setdefault(
                    "buffer_size",
                    DMA_DEFAULT_SIZES[p_type_base]["buffer"]
                )

                dma_code.append(f"static uint8_t {instance_lower}_buf[{buf_size}];")

    return "/* DMA Resources */\n" + "\n".join(dma_code) if dma_code else ""


# --------------------------
# Peripheral Generation
# --------------------------
class PeripheralFactory:
    @staticmethod
    def create(p_type: str, instance: str, config: dict) -> str:
        handler_map = {
            "ADC": PeripheralFactory._generate_adc,
            "TIM": PeripheralFactory._generate_tim,
            "FDCAN": PeripheralFactory._generate_canfd,
            "CAN": PeripheralFactory._generate_can,
            "SPI": PeripheralFactory._generate_spi,
            "USART": PeripheralFactory._generate_uart,
            "UART": PeripheralFactory._generate_uart,
            "LPUART": PeripheralFactory._generate_uart,
            "I2C": PeripheralFactory._generate_i2c
        }
        generator = handler_map.get(p_type.upper())
        return generator(instance, config) if generator else ("", "")

    @staticmethod
    def _generate_adc(instance: str, config: dict) -> tuple:
        """Generate ADC initialization with configurable queue size."""
        conversions = config.get("RegularConversions", []) if config.get("DMA") == "ENABLE" else config.get("Channels",
                                                                                                            [])
        adc_config = libxr_settings['ADC'].setdefault(instance.lower(), {})
        vref = adc_config.setdefault('vref', 3.3)

        channels = f"  std::array<uint32_t, {len(conversions)}> {instance.lower()}_channels = {{{', '.join(conversions)}}};\n"
        code = f"  STM32ADC {instance.lower()}(&h{instance.lower()}, {instance.lower()}_buf, {instance.lower()}_channels, {vref});\n"

        channels_code = channels + code
        index = 0

        for channel in conversions:
            channels_code += f"  auto {instance.lower()}_{channel.lower()} = {instance.lower()}.GetChannel({index});\n"
            channels_code += f"  UNUSED({instance.lower()}_{channel.lower()});\n"
            _register_device(f"{instance.lower()}_{channel.lower()}", "ADC")
            index = index + 1

        return ("adc", channels_code)

    @staticmethod
    def _generate_uart(instance: str, config: dict) -> tuple:
        tx_dma = config.get("DMA_TX", "DISABLE") == "ENABLE"
        rx_dma = config.get("DMA_RX", "DISABLE") == "ENABLE"
        tx_buf = f"{instance.lower()}_tx_buf" if tx_dma else "{nullptr, 0}"
        rx_buf = f"{instance.lower()}_rx_buf" if rx_dma else "{nullptr, 0}"

        uart_config = libxr_settings['USART'].setdefault(instance.lower(), {})
        tx_queue = uart_config.setdefault("tx_queue_size", 5)
        rx_queue = uart_config.setdefault("rx_queue_size", 5)

        code = f"  STM32UART {instance.lower()}(&h{instance.lower().replace("usart", "uart")},\n" \
               f"              {rx_buf}, {tx_buf}, {rx_queue}, {tx_queue});\n"
        _register_device(f"{instance.lower()}", "UART")
        return ("main", code)

    @staticmethod
    def _generate_i2c(instance: str, config: dict) -> tuple:
        """Generate I2C initialization code with dynamic buffer configuration."""
        i2c_config = libxr_settings['I2C'].setdefault(instance.lower(), {})
        dma_min_size = i2c_config.setdefault('dma_enable_min_size', 3)
        _register_device(f"{instance.lower()}", "I2C")
        return ("main",
                f"  STM32I2C {instance.lower()}(&h{instance.lower()}, {instance.lower()}_buf, {dma_min_size});\n")

    @staticmethod
    def _generate_tim(instance: str, config: dict) -> tuple:
        """Generate PWM channel instances for TIM peripherals."""
        channels = config.get('Channels', [])
        if not channels:
            return ("", "")
        code = ""
        for ch in channels:
            ch_num = ch.replace('CH', '')
            dev_name = f"pwm_{instance.lower()}_ch{ch_num}"
            code += f"  STM32PWM {dev_name}(&h{instance.lower()}, TIM_CHANNEL_{ch_num});\n"
            _register_device(dev_name, "PWM")
        return ("pwm", code)

    @staticmethod
    def _generate_canfd(instance: str, config: dict) -> tuple:
        """Generate CAN FD initialization with configurable queue size."""
        queue_size = libxr_settings['FDCAN'].get(instance, {}).get('queue_size', 5)
        _register_device(f"{instance.lower()}", "FDCAN")
        return ("main",
                f'  STM32CANFD {instance.lower()}(&h{instance.lower()}, "{instance.lower()}", {queue_size});\n')

    @staticmethod
    def _generate_can(instance: str, config: dict) -> tuple:
        """Generate classic CAN initialization with queue configuration."""
        queue_size = libxr_settings['CAN'].get(instance, {}).get('queue_size', 5)
        _register_device(f"{instance.lower()}", "CAN")
        return ("main",
                f'  STM32CAN {instance.lower()}(&h{instance.lower()}, "{instance.lower()}", {queue_size});\n')

    @staticmethod
    def _generate_spi(instance: str, config: dict) -> tuple:
        """Generate SPI initialization with DMA buffer configuration."""
        tx_enabled = config.get('DMA_TX', 'DISABLE') == 'ENABLE'
        rx_enabled = config.get('DMA_RX', 'DISABLE') == 'ENABLE'

        spi_config = libxr_settings['SPI'].setdefault(instance.lower(), {})
        dma_min_size = spi_config.setdefault('dma_enable_min_size', 3)

        tx_buf = f"{instance.lower()}_tx_buf" if tx_enabled else "{nullptr, 0}"
        rx_buf = f"{instance.lower()}_rx_buf" if rx_enabled else "{nullptr, 0}"

        _register_device(f"{instance.lower()}", "SPI")

        return ("main",
                f'  STM32SPI {instance.lower()}(&h{instance.lower()}, {rx_buf}, {tx_buf}, {dma_min_size});\n')


def _generate_header_includes(use_xrobot: bool = False) -> str:
    """Generate essential header inclusions with optional XRobot components."""
    headers = [
        '#include "app_main.h"',
        '#include "libxr.hpp"',
        '#include "main.h"',
        '#include "stm32_adc.hpp"',
        '#include "stm32_can.hpp"',
        '#include "stm32_canfd.hpp"',
        '#include "stm32_gpio.hpp"',
        '#include "stm32_i2c.hpp"',
        '#include "stm32_power.hpp"',
        '#include "stm32_pwm.hpp"',
        '#include "stm32_spi.hpp"',
        '#include "stm32_timebase.hpp"',
        '#include "stm32_uart.hpp"',
        '#include "stm32_usb.hpp"'
    ]

    if use_xrobot:
        headers.extend([
            '#include "hardware_container.hpp"'
        ])

    return '\n'.join(headers) + '\n\nusing namespace LibXR;\n'


def _generate_extern_declarations(project_data: dict) -> str:
    """Generate external declarations for HAL handlers with comprehensive checks."""
    externs = set()

    # Timebase source declaration
    timebase_cfg = project_data.get('Timebase', {})
    if timebase_cfg.get('Source', 'SysTick') != 'SysTick':
        src = timebase_cfg['Source']
        if src.startswith('TIM'):
            externs.add(f'extern TIM_HandleTypeDef h{src.lower()};')
        elif src.startswith('LPTIM'):
            externs.add(f'extern LPTIM_HandleTypeDef h{src.lower()};')
        elif src.startswith('HRTIM'):
            externs.add(f'extern HRTIM_HandleTypeDef h{src.lower()};')

    # Peripheral declarations
    peripherals = project_data.get('Peripherals', {})
    for p_type, instances in peripherals.items():
        for instance in instances:
            if p_type == 'USB':
                usb_info = _detect_usb_device(project_data)
                if usb_info is not None:
                    externs.add(f'extern USBD_HandleTypeDef {usb_info['handler']};')
                    externs.add(f'extern uint8_t UserTxBuffer{usb_info['speed']}[APP_TX_DATA_SIZE];')
                    externs.add(f'extern uint8_t UserRxBuffer{usb_info['speed']}[APP_RX_DATA_SIZE];')
            else:
                handle_type = 'UART_HandleTypeDef' if p_type in ['USART', 'UART', 'LPUART'] else f'{p_type}_HandleTypeDef'
                if p_type in ['USART', 'UART', 'LPUART']:
                    externs.add(f'extern {handle_type} h{instance.lower().replace("usart", "uart")};')
                else:
                    externs.add(f'extern {handle_type} h{instance.lower()};')

    return '/* External HAL Declarations */\n' + '\n'.join(sorted(externs)) + '\n'


def preserve_user_blocks(existing_code: str, section: int) -> str:
    """Preserve user code between protection markers with enhanced pattern matching."""
    patterns = {
        1: ('/\* User Code Begin 1 \*/(.*?)/\* User Code End 1 \*/', ''),
        2: (r'/\* User Code Begin 2 \*/(.*?)/\* User Code End 2 \*/', ''),
        3: (r'/\* User Code Begin 3 \*/(.*?)/\* User Code End 3 \*/',
            '  while(true) {\n    Thread::Sleep(UINT32_MAX);\n  }')
    }

    if section not in patterns:
        return ''

    pattern, default = patterns[section]
    match = re.search(pattern, existing_code, re.DOTALL)
    if section != 1:
        return '  ' + match.group(1).strip() if match else default
    else:
        return match.group(1).strip() if match else default


def _generate_core_system(project_data: dict) -> str:
    """Generate core system initialization with timebase configuration."""
    timebase_cfg = project_data.get('Timebase', {'Source': 'SysTick'})
    source = timebase_cfg.get('Source', 'SysTick')

    timebase_init = '  STM32Timebase timebase;'  # Default to SysTick

    if source != 'SysTick':
        timer_type = 'TIM' if source.startswith('TIM') else \
            'LPTIM' if source.startswith('LPTIM') else 'HRTIM'
        handler = f'h{source.lower()}'
        timebase_init = f'  STM32TimerTimebase timebase(&{handler});'

    system_type = libxr_settings['SYSTEM']
    timer_cfg = libxr_settings['software_timer']

    init_args = ""
    if system_type == 'None':  # Bare-metal
        init_args = ""
    elif system_type == 'FreeRTOS' or system_type == 'ThreadX':
        init_args = f"{timer_cfg['priority']}, {timer_cfg['stack_depth']}"
    else:
        logging.error(f'Unsupported system type: {system_type}')
        sys.exit(1)

    return f"""{timebase_init}
  PlatformInit({init_args});
  STM32PowerManager power_manager;"""


def generate_gpio_config(project_data: dict) -> str:
    """Generate GPIO initialization code with EXTI support."""
    code = '\n  /* GPIO Configuration */\n'
    for port, config in project_data.get('GPIO', {}).items():
        alias = generate_gpio_alias(port, config, project_data)
        code += f'  STM32GPIO {alias};\n'
    return code


# --------------------------
# Terminal Configuration
# --------------------------
def configure_terminal(project_data: dict) -> str:
    code = ""
    terminal_source = libxr_settings.get("terminal_source", "").lower()
    usb_info = _detect_usb_device(project_data)
    uart_devices = list(project_data.get("Peripherals", {}).get("USART", {}).keys())
    # User-specified terminal source
    if terminal_source != "":
        print("terminal source:", terminal_source)
        if terminal_source == "usb" and usb_info:
            print("123")
            code += _setup_usb(usb_info)
            code += _setup_usb_terminal(usb_info)
        elif terminal_source in [d.lower() for d in uart_devices]:
            dev = terminal_source.upper()
            code += f"""  STDIO::read_ = &{dev.lower()}.read_port_;
  STDIO::write_ = &{dev.lower()}.write_port_;
"""
        else:
            logging.warning(f"Invalid terminal_source: {terminal_source}")
    else:
        if usb_info:
            code += _setup_usb(usb_info)
            _register_device(f"{uart_devices[0].lower()}", "UART")

    if terminal_source != "":
        term_config = libxr_settings.setdefault("Terminal", {})
        params = [
            term_config.setdefault("READ_BUFF_SIZE", 32),
            term_config.setdefault("MAX_LINE_SIZE", 32),
            term_config.setdefault("MAX_ARG_NUMBER", 5),
            term_config.setdefault("MAX_HISTORY_NUMBER", 5)
        ]
        code += f"""\
  RamFS ramfs("XRobot");
  Terminal<{', '.join(map(str, params))}> terminal(ramfs);
  auto terminal_task = Timer::CreateTask(terminal.TaskFun, &terminal, 10);
  Timer::Add(terminal_task);
  Timer::Start(terminal_task);
"""
        _register_device("ramfs", "RamFS")
        _register_device("terminal", "Terminal")
    return code


def _setup_usb_terminal(usb_info: dict) -> str:
    return (
        "  STDIO::read_ = &uart_cdc.read_port_;\n"
        "  STDIO::write_ = &uart_cdc.write_port_;\n"
    )

def _setup_usb(usb_info: dict) -> str:
    _register_device('uart_cdc', "UART")
    return (
        f"  STM32VirtualUART uart_cdc({usb_info['handler']}, "
        f"UserTxBuffer{usb_info['speed']}, UserRxBuffer{usb_info['speed']}, 5, 5);\n"
    )


def _detect_usb_device(project_data: dict) -> dict:
    usb_config = project_data.get("Peripherals", {}).get("USB", {})
    if not usb_config:
        return None

    speed = 'FS'
    mode = "Device"

    for instance, config in project_data.get("Peripherals", {}).get("USB", {}).items():
        if 'HS' in instance:
            speed = 'HS'

        if 'FS' in instance:
            speed = 'FS'

        # if 'OTG' in instance:
        #     mode = 'OTG'

    return {
        "handler": f"hUsb{mode}{speed}",
        "speed": speed
    }

# --------------------------
# XRobot Integration
# --------------------------
def generate_xrobot_hardware_container() -> str:
    """
    Generate a C++ definition for HardwareContainer using Entry<T> syntax.
    Each device is associated with its logical aliases.
    """
    global device_aliases

    # Normalize device_aliases structure
    libxr_settings["device_aliases"] = {
        dev: {
            "type": meta.get("type", "Unknown"),
            "aliases": sorted(set(meta.get("aliases", [])))
        }
        for dev, meta in device_aliases.items()
    }

    # Collect types (Entry<T>) and entries (device with aliases)
    type_list = []
    entry_list = []

    for dev, meta in device_aliases.items():
        dev_type = meta["type"]
        aliases = meta["aliases"]

        type_list.append(f"    LibXR::Entry<LibXR::{dev_type}>")

        if not aliases:
            entry_list.append(f"  {{{dev}, {{}}}}")  # No aliases
        else:
            alias_str = ", ".join(f'"{alias}"' for alias in aliases)
            entry_list.append(f"  {{{dev}, {{{alias_str}}}}}")  # With aliases

    if not type_list:
        return "// No devices to generate HardwareContainer.\n"

    return f"""\nLibXR::HardwareContainer<\n{',\n'.join(type_list)}\n> peripherals{{\n{",\n".join(entry_list)}\n}};\n"""

# --------------------------
# Main Generator
# --------------------------
def generate_full_code(project_data: dict, use_xrobot: bool, existing_code: str) -> str:
    components = [
        _generate_header_includes(use_xrobot),
        '/* User Code Begin 1 */',
        preserve_user_blocks(existing_code, 1),
        '/* User Code End 1 */',
        _generate_extern_declarations(project_data),

        generate_dma_resources(project_data),

        '\nextern "C" void app_main(void) {',
        '  /* User Code Begin 2 */',
        preserve_user_blocks(existing_code, 2),
        '  /* User Code End 2 */',
        _generate_core_system(project_data),
        generate_gpio_config(project_data),
        generate_peripheral_instances(project_data),
        configure_terminal(project_data),
        generate_xrobot_hardware_container() if use_xrobot else '',
        '  /* User Code Begin 3 */',
        preserve_user_blocks(existing_code, 3),
        '  /* User Code End 3 */',
        '}'
    ]
    return '\n'.join(filter(None, components))


def generate_app_main_header(output_dir: str) -> None:
    """Generate app_main.h header file."""
    header_path = os.path.join(output_dir, "app_main.h")
    content = """#ifdef __cplusplus
extern "C" {
#endif

void app_main(void);

#ifdef __cplusplus
}
#endif
"""

    if not os.path.exists(header_path) or open(header_path).read() != content:
        with open(header_path, "w", encoding="utf-8") as f:
            f.write(content)
        logging.info(f"Generated header: {header_path}")


def main():
    try:
        args = parse_arguments()

        # Load configurations
        project_data = load_configuration(args.input, args.xrobot)
        load_libxr_config(os.path.dirname(args.output))
        initialize_device_aliases(args.xrobot)

        output_dir = os.path.dirname(args.output)
        os.makedirs(output_dir, exist_ok=True)

        # Generate code
        existing_code = ""
        if os.path.exists(args.output):
            with open(args.output, "r", encoding="utf-8") as f:
                existing_code = f.read()

        output_code = generate_full_code(project_data, args.xrobot, existing_code)

        # Write output
        with open(args.output, "w", encoding="utf-8") as f:
            f.write(output_code)

        config_path = os.path.join(output_dir, "libxr_config.yaml")

        with open(config_path, "w", encoding="utf-8") as f:
            cleaned_config = {
                k: v for k, v in libxr_settings.items()
                if not (isinstance(v, dict) and len(v) == 0)
                   and not (k == "device_aliases" and not args.xrobot)
            }
            yaml.dump(cleaned_config, f, allow_unicode=True, sort_keys=False)

        logging.info(f"Successfully generated: {output_dir}")

        generate_app_main_header(output_dir)
        logging.info("Generated header file: app_main.h")

    except Exception as e:
        logging.error(f"Generation failed: {str(e)}")
        sys.exit(1)


if __name__ == "__main__":
    main()
