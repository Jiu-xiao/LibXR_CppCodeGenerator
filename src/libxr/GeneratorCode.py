#!/usr/bin/env python

import argparse
import json
import os
import sys

libxr_config = {
    "terminal_source": "",
    "software_timer": {"priority": 2, "stack_depth": 512},
    "SPI": {},
    "I2C": {},
    "USART": {},
    "USB": {},
    "ADC": {},
    "TIM": {},
    "CAN": {},
    "FDCAN": {},
    "SYSTEM" : "None"
}


def load_json(file_path):
    """Load JSON configuration file."""
    with open(file_path, "r", encoding="utf-8") as f:
        return json.load(f)


def gpio_alias(port, gpio_data):
    """Generate GPIO configuration code, supporting CubeMX macros and unaliased cases."""
    label = gpio_data.get("Label")  # Could be empty
    is_exti = gpio_data.get("GPXTI", False)  # Whether it's an EXTI interrupt pin

    # **No alias** → Directly use GPIOx, GPIO_PIN_x
    parts = port.split("-")
    port_define = f"GPIO{parts[0][1]}"  # E.g., "PA4" → "GPIOA"
    pin_define = f"GPIO_PIN_{parts[0][2:]}"  # E.g., "PA4" → "GPIO_PIN_4"

    if label:
        pin_define = f"{label}_Pin"
        port_define = f"{label}_GPIO_Port"

    # Calculate EXTI_IRQn
    pin_num = int(parts[0][2:])
    if is_exti:
        if 5 <= pin_num <= 9:
            irq_define = "EXTI9_5_IRQn"
        elif 10 <= pin_num <= 15:
            irq_define = "EXTI15_10_IRQn"
        else:
            irq_define = f"EXTI{pin_num}_IRQn"
    else:
        irq_define = None

    # **Final formatted output**
    if irq_define:
        return f"{label or port}({port_define}, {pin_define}, {irq_define})"
    return f"{label or port}({port_define}, {pin_define})"


def generate_dma_buffers(periph, instance, buffer_sizes):
    """Generate DMA buffers for peripherals requiring DMA."""
    if periph in ["SPI", "USART"]:
        # Get USART configuration
        tx_buffer_size = None
        rx_buffer_size = None
        if libxr_config.get(periph):
            buffer_size = libxr_config[periph].get(instance, None)
        else:
            buffer_size = None

        if buffer_size is None:
            rx_buffer_size = buffer_sizes[periph]
            tx_buffer_size = buffer_sizes[periph]
            if periph == "USART":
                libxr_config[periph][instance] = {
                    "tx_buffer_size": tx_buffer_size,
                    "rx_buffer_size": rx_buffer_size,
                    "tx_queue_size": 5,
                    "rx_queue_size": 5,
                }
            else:
                libxr_config[periph][instance] = {
                    "tx_buffer_size": tx_buffer_size,
                    "rx_buffer_size": rx_buffer_size,
                }
        else:
            tx_buffer_size = buffer_size["tx_buffer_size"]
            rx_buffer_size = buffer_size["rx_buffer_size"]

        return f"static uint8_t {instance.lower()}_buff_tx[{tx_buffer_size}], {instance.lower()}_buff_rx[{rx_buffer_size}];\n"
    if periph in ["I2C", "ADC"]:
        # Get I2C configuration
        if libxr_config.get(periph):
            buffer_size = libxr_config[periph].get(instance, None)
        else:
            buffer_size = None

        if buffer_size is None:
            buffer_size = buffer_sizes[periph]
            libxr_config[periph][instance] = {"buffer_size": buffer_size}
        else:
            buffer_size = buffer_size["buffer_size"]
        return f"static uint8_t {instance.lower()}_buffer[{buffer_size}];\n"
    return ""  # Other peripherals don't need DMA


def generate_gpio_config(project_data):
    """Generate GPIO configuration code."""
    gpio_section = "\n  /* GPIO Configuration */\n"
    for port, config in project_data["GPIO"].items():
        gpio_section += f"  LibXR::STM32GPIO {gpio_alias(port, config)};\n"
    return gpio_section


def get_system_config(project_data):
    if project_data.get("FreeRTOS"):
        return "FreeRTOS"
    else:
        return "None"

def generate_extern_config(project_data, buffer_sizes):
    """Generate peripheral instantiation code and allocate DMA buffers if needed."""
    dma_section = "\n/* DMA Buffers */\n"
    externs = set()

    for periph, instances in project_data["Peripherals"].items():
        for instance, config in instances.items():
            # Handle extern definitions
            if periph == "USART" or periph == "UART":
                externs.add(
                    f"extern UART_HandleTypeDef h{instance.lower().replace('usart', 'uart')};"
                )
            elif periph == "USB":
                mode = config.get("Mode", "")
                print(instance, mode)
                if "HS" in mode.upper() or "HS" in instance.upper():
                    usb_speed = "HS"
                else:
                    usb_speed = "FS"

                if "DEVICE_ONLY" in mode.upper() or "DEVICE" in instance.upper():
                    usb_mode = "Device"
                else:
                    usb_mode = "Otg"

                externs.add(f"extern USBD_HandleTypeDef hUsb{usb_mode}{usb_speed};")
                externs.add(f"""extern uint8_t UserRxBuffer{usb_speed}[APP_RX_DATA_SIZE];
extern uint8_t UserTxBuffer{usb_speed}[APP_TX_DATA_SIZE];
""")

            else:
                externs.add(f"extern {periph}_HandleTypeDef h{instance.lower()};")

            # Generate DMA Buffers
            if periph not in ["TIM", "USB", "CAN", "FDCAN"]:
                dma_section += generate_dma_buffers(periph, instance, buffer_sizes)

    return "\n".join(sorted(externs)) + "\n" + dma_section


def generate_peripherals_config(project_data):
    """Generate peripheral instantiation code and assign DMA buffers."""
    periph_section = "\n  /* Peripheral Configuration */\n"
    adc_channels = ""
    pwm_section = ""

    for periph, instances in project_data["Peripherals"].items():
        for instance, config in instances.items():

            # ADC
            if periph == "ADC":
                dma_enabled = (
                        config.get("DMA", "DISABLE") == "ENABLE"
                )  # Default "DISABLE" if missing
                conversion_key = "RegularConversions" if dma_enabled else "Channels"

                # Ensure conversions are in list format
                conversions = config.get(conversion_key, [])
                if isinstance(conversions, str):
                    conversions = list(eval(conversions))  # Parse string safely

                adc_channels += (
                    f"  std::array<uint32_t, {len(conversions)}> {instance.lower()}_channels = "
                    f"{{{', '.join(conversions)}}};\n"
                )
                periph_section += (
                    f"  LibXR::STM32ADC {instance.lower()}(&h{instance.lower()}, "
                    f"RawData({instance.lower()}_buffer), {instance.lower()}_channels, 3.3f);\n"
                )
            elif periph == "FDCAN":
                config = libxr_config[periph].get(instance, None)

                if config is None:
                    config = {"queue_size": 5}
                    libxr_config[periph][instance] = config

                queue_size = config.get("queue_size", None)

                if queue_size is None:
                    queue_size = 5
                    libxr_config[periph][instance]["queue_size"] = queue_size

                periph_section += (
                    f"  LibXR::STM32CANFD {instance.lower()}(&h{instance.lower()}, "
                    f'"{instance.lower()}", {queue_size});\n'
                )
            elif periph == "CAN":
                config = libxr_config[periph].get(instance, None)

                if config is None:
                    config = {"queue_size": 5}
                    libxr_config[periph][instance] = config

                queue_size = config.get("queue_size", None)

                if queue_size is None:
                    queue_size = 5
                    libxr_config[periph][instance]["queue_size"] = queue_size
                periph_section += f'  LibXR::STM32CAN {instance.lower()}(&h{instance.lower()}, "{instance.lower()}", {queue_size});\n'
            elif periph == "SPI":
                tx_dma_enabled = config.get("DMA_TX", "DISABLE") == "ENABLE"
                rx_dma_enabled = config.get("DMA_RX", "DISABLE") == "ENABLE"

                tx_buffer = (
                    f"{instance.lower()}_buff_tx" if tx_dma_enabled else "{nullptr, 0}"
                )
                rx_buffer = (
                    f"{instance.lower()}_buff_rx" if rx_dma_enabled else "{nullptr, 0}"
                )

                periph_section += f"  LibXR::STM32{periph} {instance.lower()}(&h{instance.lower()}, {rx_buffer}, {tx_buffer});\n"
            elif periph == "USART":
                tx_dma_enabled = config.get("DMA_TX", "DISABLE") == "ENABLE"
                rx_dma_enabled = config.get("DMA_RX", "DISABLE") == "ENABLE"

                config = libxr_config["USART"][instance]

                tx_buffer = (
                    f"{instance.lower()}_buff_tx" if tx_dma_enabled else "{nullptr, 0}"
                )
                rx_buffer = (
                    f"{instance.lower()}_buff_rx" if rx_dma_enabled else "{nullptr, 0}"
                )

                tx_queue_size = config.get("tx_queue_size", None) if tx_dma_enabled else 0
                rx_queue_size = config.get("rx_queue_size", None) if rx_dma_enabled else 0

                if tx_queue_size is None:
                    tx_queue_size = 5
                    libxr_config["USART"][instance]["tx_queue_size"] = tx_queue_size
                if rx_queue_size is None:
                    rx_queue_size = 5
                    libxr_config["USART"][instance]["rx_queue_size"] = rx_queue_size

                periph_section += (
                    (
                        f"  LibXR::STM32{periph} {instance.lower()}(&h{instance.lower()}, {rx_buffer}, {tx_buffer}, {rx_queue_size}, {tx_queue_size});\n"
                    )
                    .replace("USART", "UART")
                    .replace("husart", "huart")
                )
            elif periph == "I2C":
                periph_section += f"  LibXR::STM32I2C {instance.lower()}(&h{instance.lower()}, {instance.lower()}_buffer);\n"
            elif periph == "TIM" and "Channels" in instances[instance]:
                for channel in instances[instance]["Channels"]:
                    channel_num = channel.replace("CH", "")
                    pwm_section += f"  LibXR::STM32PWM pwm_{instance.lower()}_ch{channel_num}(&h{instance.lower()}, TIM_CHANNEL_{channel_num});\n"

    return "\n" + adc_channels + pwm_section + periph_section


def generate_terminal_config(project_data, terminal_source):
    """Generate Terminal configuration based on peripheral settings."""
    usb_device = None
    for instance, config in project_data["Peripherals"].get("USB", {}).items():
        mode = config.get("Mode", "")
        if "HS" in mode.upper() or "HS" in instance.upper():
            usb_speed = "HS"
        else:
            usb_speed = "FS"

        if "DEVICE_ONLY" in mode.upper() or "DEVICE" in instance.upper():
            usb_mode = "Device"
        else:
            usb_mode = "Otg"

        usb_device = f"hUsb{usb_mode}{usb_speed}"

    # Get all UART devices
    uart_list = list(project_data["Peripherals"].get("USART", {}).keys()) + list(
        project_data["Peripherals"].get("UART", {}).keys()
    )

    terminal_config = ""

    if usb_device:
        terminal_config = f"  LibXR::STM32VirtualUART uart_cdc({usb_device}, UserTxBuffer{usb_speed}, UserRxBuffer{usb_speed}, 5, 5);\n"

    if terminal_source != "":
        terminal_config += f"  STDIO::read_ = &{terminal_source}.read_port_;\n"
        terminal_config += f"  STDIO::write_ = &{terminal_source}.write_port_;\n"
    elif usb_device:
        terminal_config += "  STDIO::read_ = &uart_cdc.read_port_;\n"
        terminal_config += "  STDIO::write_ = &uart_cdc.write_port_;\n"
    elif uart_list:
        first_uart = uart_list[0].lower()
        terminal_config = f"  STDIO::read_ = &{first_uart}.read_port_;\n"
        terminal_config += f"  STDIO::write_ = &{first_uart}.write_port_;\n"

    if terminal_config:
        terminal_config += "  RamFS ramfs(\"XRobot\");\n"
        terminal_config += "  Terminal terminal(ramfs);\n"
        terminal_config += "  auto terminal_task = Timer::CreateTask(terminal.TaskFun, &terminal, 10);\n"
        terminal_config += "  Timer::Add(terminal_task);\n"
        terminal_config += "  Timer::Start(terminal_task);\n"

    return terminal_config


def preserve_user_code(existing_code, section):
    """Preserve user code blocks between markers."""
    start_marker = f"/* User Code Begin {section} */"
    end_marker = f"/* User Code End {section} */"

    if start_marker in existing_code and end_marker in existing_code:
        preserved_code = existing_code.split(start_marker, 1)[1].split(end_marker, 1)[0]
        return preserved_code.strip()

    if section == 3:
        return """
  while (true) {
    Thread::Sleep(UINT32_MAX);
  }
"""
    else:
        return ""  # Return empty to preserve code structure


def generate_cpp_code(
        project_data, project_name, terminal_source, buffer_sizes, existing_code=""
):
    timer_config = libxr_config.get("software_timer", (None, None))
    if timer_config is None:
        timer_pri = 2
        timer_stack_depth = 512
        libxr_config["software_timer"] = {
            "priority": timer_pri,
            "stack_depth": timer_stack_depth,
        }
    else:
        timer_pri = timer_config["priority"]
        timer_stack_depth = timer_config["stack_depth"]

    libxr_config["SYSTEM"] = get_system_config(project_data)

    if libxr_config.get("SYSTEM") is "None":
        paltform_init_args = ""
    else:
        paltform_init_args = f"{timer_pri}, {timer_stack_depth}"

    print(f"System: {libxr_config.get('SYSTEM')}")

    """Generate complete C++ code."""
    cpp_code_include = f"""#include \"app_main.h\"
#include \"database.hpp\"
#include \"libxr.hpp\"
#include \"main.h\"
#include \"stm32_adc.hpp\"
#include \"stm32_can.hpp\"
#include \"stm32_canfd.hpp\"
#include \"stm32_gpio.hpp\"
#include \"stm32_i2c.hpp\"
#include \"stm32_power.hpp\"
#include \"stm32_pwm.hpp\"
#include \"stm32_spi.hpp\"
#include \"stm32_timebase.hpp\"
#include \"stm32_uart.hpp\"
#include \"stm32_usb.hpp\"

using namespace LibXR;
""" + generate_extern_config(
        project_data, buffer_sizes
    )

    cpp_code = (
            cpp_code_include
            + f"""
/* User Code Begin 1 */
"""
            + preserve_user_code(existing_code, 1)
            + """
/* User Code End 1 */

extern \"C\" void app_main(void) {
  /* User Code Begin 2 */
"""
            + preserve_user_code(existing_code, 2)
            + f"""
  /* User Code End 2 */

  LibXR::STM32Timebase stm32_timebase;
  LibXR::PlatformInit({paltform_init_args});
  LibXR::STM32PowerManager power_manager;
"""
    )

    cpp_code += generate_gpio_config(project_data)
    cpp_code += generate_peripherals_config(project_data)
    cpp_code += generate_terminal_config(project_data, terminal_source)

    cpp_code += (
            """
  /* User Code Begin 3 */
  """
            + preserve_user_code(existing_code, 3)
            + """
  /* User Code End 3 */
}
""")

    return cpp_code


def parse_arguments():
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Generate C++ code from JSON input with configurable DMA buffer sizes."
    )

    parser.add_argument(
        "-i", "--input", type=str, required=True, default=24, help="Input file path."
    )
    parser.add_argument(
        "-o", "--output", type=str, default=24, help="Output file path."
    )

    return parser.parse_args()


def generate_app_main_header(output_file):
    """Generate app_main.h header file."""
    with open(output_file, "w", encoding="utf-8") as f:
        f.write(
            """#ifdef __cplusplus
extern "C" {
#endif

void app_main(void); // NOLINT

#ifdef __cplusplus
}
#endif
"""
        )


def main():
    global libxr_config

    args = parse_arguments()

    # Parse input and output paths
    input_file = args.input
    output_file = (
        args.output if args.output else os.path.splitext(input_file)[0] + ".cpp"
    )

    # Parse buffer_sizes
    buffer_sizes = {
        "SPI": 32,
        "USART": 128,
        "I2C": 32,
        "ADC": 32,
    }

    libxr_config_path = (
            os.path.dirname(os.path.abspath(output_file)) + "/libxr_config.json"
    )
    print(libxr_config_path)

    if os.path.exists(libxr_config_path):
        try:
            with open(libxr_config_path, "r", encoding="utf-8") as f:
                libxr_config = json.load(f)
        except (json.JSONDecodeError, IOError) as e:
            print(f"Failed to load config: {e}, saving default config.")
            with open(libxr_config_path, "w", encoding="utf-8") as f:
                json.dump(libxr_config, f, indent=4, ensure_ascii=False)
    else:
        os.makedirs(os.path.dirname(libxr_config_path), exist_ok=True)
        with open(libxr_config_path, "w", encoding="utf-8") as f:
            json.dump(libxr_config, f, indent=4, ensure_ascii=False)

    terminal_source = libxr_config.get("terminal_source", "")

    # Read JSON data
    try:
        with open(input_file, "r", encoding="utf-8") as f:
            project_data = json.load(f)
    except FileNotFoundError:
        print(f"Error: File {input_file} not found.")
        sys.exit(1)
    except json.JSONDecodeError:
        print(f"Error: Failed to parse JSON file {input_file}.")
        sys.exit(1)

    # Read existing code (if any)
    existing_code = ""
    if os.path.exists(output_file):
        with open(output_file, "r", encoding="utf-8") as f:
            existing_code = f.read()

    # Generate C++ code
    cpp_code = generate_cpp_code(
        project_data,
        os.path.splitext(os.path.basename(input_file))[0],
        terminal_source,
        buffer_sizes,
        existing_code,
    )

    # Write to output file
    with open(output_file, "w", encoding="utf-8") as f:
        f.write(cpp_code)

    print(f"{output_file} generated successfully!")

    # Generate app_main.h
    header_file = os.path.join(os.path.dirname(output_file), "app_main.h")
    with open(header_file, "w", encoding="utf-8") as f:
        f.write("void app_main(void);\n")

    generate_app_main_header(header_file)
    print(f"{header_file} generated successfully!")

    with open(libxr_config_path, "w", encoding="utf-8") as f:
        # Write the JSON data (libxr_config) to the file
        json.dump(libxr_config, f, indent=4, ensure_ascii=False)
