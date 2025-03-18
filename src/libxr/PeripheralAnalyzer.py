#!/usr/bin/env python

import argparse
import json
import os
import re
from collections import defaultdict


def get_peripheral_defaults(p_type):
    """Return default configurations for a peripheral type."""
    defaults = {
        "TIM": {"Mode": None, "ClockPrescaler": None, "Period": None, "Prescaler": None, "Channels": {}, "Pulses": {}},
        "ADC": {"ClockPrescaler": None, "Resolution": None, "DataAlignment": None, "ContinuousMode": None,
                "RegularConversions": [], "SamplingTime": {}},
        "SPI": {"BaudRate": None, "Direction": None, "CLKPolarity": None, "CLKPhase": None},
        "I2C": {"ClockSpeed": None, "DutyCycle": None, "AddressingMode": None, "DualAddressMode": None,
                "GeneralCallMode": None, "NoStretchMode": None},
        "USART": {"BaudRate": None, "WordLength": None, "Parity": None, "StopBits": None},
        "USB": {"Mode": None, "Speed": None, "VBus": None, "Endpoints": {}},
        "DEFAULT": {"Parameters": {}}
    }
    return defaults.get(p_type, defaults["DEFAULT"]).copy()


def parse_ioc_file(ioc_path):
    """Parse .ioc file to extract peripheral configurations."""
    if not os.path.exists(ioc_path):
        print(f"[Error] File {ioc_path} not found")
        return None

    peripherals = defaultdict(lambda: defaultdict(dict))
    gpio_pins = defaultdict(dict)
    dma_requests = {}
    dma_configs = defaultdict(list)
    freertos_config = {"Tasks": {}, "Heap": None, "Features": {}}

    # Read raw key-value pairs
    raw_map = {}
    with open(ioc_path, "r", encoding="utf-8") as f:
        for line in f:
            if "=" in line and not line.startswith("#"):
                key, value = map(str.strip, line.split("=", 1))
                raw_map[key.replace("\\#", "")] = value

    # Parse GPIO configurations
    gpio_pattern = re.compile(r"^(P[A-I]\d+(?:-[\w]+)*)\.(Signal|GPIO_Label|GPIO_PuPd)")
    for key, value in raw_map.items():
        if match := gpio_pattern.match(key):
            pin, prop = match.groups()
            prop_map = {
                "Signal": ("Signal", value),
                "GPIO_Label": ("Label", value),
                "GPIO_PuPd": ("Pull", value),
            }
            field, val = prop_map[prop]
            gpio_pins[pin][field] = val
            if "GPXTI" in value:
                gpio_pins[pin]["GPXTI"] = True

    # Parse peripherals
    for key, value in raw_map.items():
        parts = key.split(".")
        if len(parts) < 2:
            continue

        p_name = parts[0]
        p_prop = parts[1]

        # CAN/FDCAN parsing
        if p_name.startswith(("CAN", "FDCAN")):
            p_type = "FDCAN" if p_name.startswith("FDCAN") else "CAN"
            can = p_name
            if not peripherals[p_type].get(can):
                peripherals[p_type][can] = get_peripheral_defaults(p_type)

            if "CalculateBaudRate" in p_prop:
                peripherals[p_type][can]["BaudRate"] = value
            elif "Mode" in p_prop:
                peripherals[p_type][can]["Mode"] = value

            # Legacy CAN parameters
            if p_type == "CAN":
                if "BS1" in p_prop:
                    peripherals[p_type][can]["TimeSeg1"] = value
                elif "BS2" in p_prop:
                    peripherals[p_type][can]["TimeSeg2"] = value
                elif "ABOM" in p_prop:
                    peripherals[p_type][can]["AutoRetransmission"] = value == "ENABLE"
                elif "AWUM" in p_prop:
                    peripherals[p_type][can]["AutoWakeup"] = value == "ENABLE"

            # FDCAN parameters
            if p_type == "FDCAN":
                if "NominalPrescaler" in p_prop:
                    peripherals[p_type][can]["NominalBaudRate"] = value
                elif "DataPrescaler" in p_prop:
                    peripherals[p_type][can]["DataBaudRate"] = value
                elif "FrameFormat" in p_prop:
                    peripherals[p_type][can]["FrameFormat"] = value
                elif "StdFiltersNbr" in p_prop:
                    peripherals[p_type][can]["StdFilters"] = int(value)
                elif "ExtFiltersNbr" in p_prop:
                    peripherals[p_type][can]["ExtFilters"] = int(value)

        # TIM parsing
        if p_name.startswith("TIM"):
            p_type = "TIM"
            tim = p_name
            if not peripherals[p_type].get(tim):
                peripherals[p_type][tim] = get_peripheral_defaults(p_type)

            if "Channel-PWM" in key:
                if ch_match := re.search(r"CH(\d+)", p_prop):
                    ch = f"CH{ch_match.group(1)}"
                    label = gpio_pins.get(pin, {}).get("Label", pin)
                    is_pwm = True

                    if not isinstance(peripherals[p_type][tim]["Channels"], dict):
                        peripherals[p_type][tim]["Channels"] = {}

                    peripherals[p_type][tim]["Channels"][ch] = {
                        "Label": label,
                        "PWM": is_pwm
                    }
            elif "Period" in p_prop:
                peripherals[p_type][tim]["Period"] = (
                    int(value) if value.isdigit() else value
                )
            elif "Prescaler" in p_prop:
                peripherals[p_type][tim]["Prescaler"] = (
                    int(value) if value.isdigit() else value
                )

        # ADC parsing
        elif p_name.startswith("ADC"):
            p_type = "ADC"
            adc = p_name
            if not peripherals[p_type].get(adc):
                peripherals[p_type][adc] = get_peripheral_defaults(p_type)

            if "Channel-" in p_prop and "ChannelRegularConversion" in key:
                if "Channels" not in peripherals[p_type][adc]:
                    peripherals[p_type][adc]["Channels"] = set()
                if "RegularConversions" not in peripherals[p_type][adc]:
                    peripherals[p_type][adc]["RegularConversions"] = []

                if value.startswith("ADC_CHANNEL"):
                    peripherals[p_type][adc]["Channels"].add(value)
                    peripherals[p_type][adc]["RegularConversions"].append(value)
            elif "ClockPrescaler" in p_prop:
                peripherals[p_type][adc]["ClockPrescaler"] = value
            elif "ContinuousConvMode" in p_prop:
                peripherals[p_type][adc]["ContinuousMode"] = value == "ENABLE"

        # SPI parsing
        elif p_name.startswith("SPI"):
            p_type = "SPI"
            spi = p_name
            if not peripherals[p_type].get(spi):
                peripherals[p_type][spi] = get_peripheral_defaults(p_type)

            if "CalculateBaudRate" in p_prop:
                peripherals[p_type][spi]["BaudRate"] = value
            elif "Direction" in p_prop:
                peripherals[p_type][spi]["Direction"] = value
            elif "CLKPolarity" in p_prop:
                peripherals[p_type][spi]["CLKPolarity"] = value
            elif "CLKPhase" in p_prop:
                peripherals[p_type][spi]["CLKPhase"] = value

        # I2C parsing
        elif p_name.startswith("I2C"):
            p_type = "I2C"
            i2c = p_name
            if not peripherals[p_type].get(i2c):
                peripherals[p_type][i2c] = get_peripheral_defaults(p_type)

            if "ClockSpeed" in p_prop:
                peripherals[p_type][i2c]["ClockSpeed"] = value
            elif "DutyCycle" in p_prop:
                peripherals[p_type][i2c]["DutyCycle"] = value
            elif "AddressingMode" in p_prop:
                peripherals[p_type][i2c]["AddressingMode"] = value
            elif "DualAddressMode" in p_prop:
                peripherals[p_type][i2c]["DualAddressMode"] = value

        # USART parsing
        elif p_name.startswith("USART") or p_name.startswith("UART"):
            p_type = "USART"
            usart = p_name
            if not peripherals[p_type].get(usart):
                peripherals[p_type][usart] = get_peripheral_defaults(p_type)

            if "BaudRate" in p_prop:
                peripherals[p_type][usart]["BaudRate"] = value
            elif "WordLength" in p_prop:
                peripherals[p_type][usart]["WordLength"] = value
            elif "Parity" in p_prop:
                peripherals[p_type][usart]["Parity"] = value
            elif "StopBits" in p_prop:
                peripherals[p_type][usart]["StopBits"] = value

        # USB parsing
        elif p_name.startswith("USB"):
            p_type = "USB"
            usb = p_name
            if not peripherals[p_type].get(usb):
                peripherals[p_type][usb] = get_peripheral_defaults(p_type)

            if "Mode" in p_prop:
                peripherals[p_type][usb]["Mode"] = value
            elif "Speed" in p_prop:
                peripherals[p_type][usb]["Speed"] = value

        # DMA parsing
        elif p_name.startswith("Dma"):
            if p_prop.startswith("Request"):
                dma_requests[p_prop[7:]] = value
            elif len(parts) > 2:
                dma_instance = parts[1]
                dma_prop = parts[2]
                config = {
                    "Instance": dma_instance,
                    "Property": dma_prop,
                    "Value": value,
                }
                dma_configs[dma_instance].append(config)

        # FreeRTOS parsing
        elif p_name == "FREERTOS":
            if p_prop.startswith("Tasks"):
                task_data = [x for x in value.split(",") if x and x != "NULL"]
                if len(task_data) >= 5:
                    task_name = task_data[0]
                    freertos_config["Tasks"][task_name] = {
                        "Priority": task_data[1],
                        "StackSize": task_data[2] + "B",
                        "EntryFunction": task_data[3],
                        "Type": task_data[4],
                    }
            elif "configTOTAL_HEAP_SIZE" in key:
                freertos_config["Heap"] = value
            elif "INCLUDE_" in key:
                freertos_config["Features"][p_prop] = value == "1"

    # Process DMA requests
    for _, request in dma_requests.items():
        peripheral, direction = request.rsplit("_", 1) if "_" in request else (request, None)

        for category in ["USART", "SPI", "ADC", "I2C"]:
            if peripheral in peripherals.get(category, {}):
                if direction:
                    dma_key = f"DMA_{direction}"
                    peripherals[category][peripheral][dma_key] = "ENABLE"
                else:
                    peripherals[category][peripheral]["DMA"] = "ENABLE"

    # Clean nested structures
    def clean_structure(obj):
        """Recursively remove empty values from nested structures."""
        if isinstance(obj, dict):
            return {k: clean_structure(v) for k, v in obj.items() if v not in (None, "", [], {})}
        if isinstance(obj, list):
            return [clean_structure(v) for v in obj if v not in (None, "", [], {})]
        return obj

    sorted_data = {
        "GPIO": clean_structure(
            {
                pin: config
                for pin, config in gpio_pins.items()
                if config.get("Signal") in {"GPIO_Output", "GPIO_Input"}
                   or config.get("Signal", "").startswith("GPXTI")
            }
        ),
        "Peripherals": clean_structure(
            {p_type: {p: cfg for p, cfg in p_group.items()} for p_type, p_group in peripherals.items()}
        ),
        "DMA": clean_structure({"Requests": dma_requests, "Configurations": dma_configs}),
        "FreeRTOS": clean_structure(freertos_config),
    }

    return sorted_data


def print_summary(data):
    """Print a summary of the parsed configuration."""
    print("\n===== Configuration Summary =====")

    # GPIO stats
    gpio_counts = defaultdict(int)
    for pin_cfg in data["GPIO"].values():
        signal_type = pin_cfg.get("Signal", "")
        if "Output" in signal_type:
            gpio_counts["Output"] += 1
        elif "Input" in signal_type:
            gpio_counts["Input"] += 1
        if pin_cfg.get("GPXTI"):
            gpio_counts["External Interrupt"] += 1
    print(f"\nGPIO Total: {len(data['GPIO'])}")
    print(f"  Output Pins: {gpio_counts.get('Output', 0)}")
    print(f"  Input Pins: {gpio_counts.get('Input', 0)}")
    print(f"  External Interrupts: {gpio_counts.get('External Interrupt', 0)}")

    # Peripheral stats
    print("\nPeripherals:")
    for p_type, p_group in data.get("Peripherals", {}).items():
        print(f"  {p_type} Count: {len(p_group)}")
        for p_name, cfg in p_group.items():
            print(f"    {p_name}:")
            if p_type == "TIM":
                if cfg.get("Channels"):
                    print(f"      Channels: {len(cfg['Channels'])}")
                if cfg.get("Period"):
                    print(f"      Period: {cfg['Period']} | Prescaler: {cfg.get('Prescaler', 'N/A')}")
            elif p_type == "ADC":
                print(f"      Continuous Mode: {'Yes' if cfg.get('ContinuousMode') else 'No'}")
                print(f"      Channels: {len(cfg.get('RegularConversions', []))}")
            elif p_type == "SPI":
                if cfg.get("BaudRate"):
                    print(f"      Baud Rate: {cfg['BaudRate']}")
                if cfg.get("Direction"):
                    print(f"      Mode: {cfg['Direction']}")
            elif p_type == "I2C":
                if cfg.get("ClockSpeed"):
                    print(f"      Clock Speed: {cfg['ClockSpeed']}")
                print(f"      No-Stretch Mode: {'Yes' if cfg.get('NoStretchMode') else 'No'}")
            elif p_type == "USART":
                if cfg.get("BaudRate"):
                    print(f"      Baud Rate: {cfg['BaudRate']}")
                if cfg.get("WordLength"):
                    print(f"      Data Bits: {cfg['WordLength']}")
                print(f"      Parity: {'Yes' if cfg.get('Parity') else 'No'}")
                if cfg.get("StopBits"):
                    print(f"      Stop Bits: {cfg['StopBits']}")
            elif p_type == "USB":
                print(f"      Mode: {cfg.get('Mode', 'N/A')}")
                print(f"      Speed: {cfg.get('Speed', 'N/A')}")

    # DMA stats
    dma_requests = data.get("DMA", {}).get("Requests", {})
    print(f"\nDMA Requests: {len(dma_requests)}")
    if dma_requests:
        print("  Request List:")
        for req_id, req_name in dma_requests.items():
            print(f"    Request {req_id}: {req_name}")

    # FreeRTOS stats
    if "FreeRTOS" in data:
        print("\nFreeRTOS Configuration:")
        print(f"  Heap Size: {data['FreeRTOS'].get('Heap', 'Unknown')}")
        if tasks := data["FreeRTOS"].get("Tasks"):
            print(f"  Task Count: {len(tasks)}")
            for name, cfg in tasks.items():
                print(f"    {name}:")
                print(f"      Priority: {cfg.get('Priority', 'Unknown')}")
                print(f"      Stack Size: {cfg.get('StackSize', 'Unknown')}")
                print(f"      Entry Function: {cfg.get('EntryFunction', 'Unknown')}")

        enabled_features = [
            feat.replace("INCLUDE_", "")
            for feat, enabled in data["FreeRTOS"].get("Features", {}).items()
            if enabled
        ]
        if enabled_features:
            print("  Enabled Features:")
            for feat in enabled_features:
                print(f"    - {feat}")


def save_to_json(data, output_path="parsed_ioc.json"):
    """Save parsed configuration to JSON file."""
    try:
        with open(output_path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False, default=str)
        print(f"\n[Pass] Configuration saved to: {output_path}")
    except Exception as e:
        print(f"[Error] Save failed: {e}")


def main():
    parser = argparse.ArgumentParser(
        description="Parse STM32Cube .ioc files in a directory and generate JSON configuration.")
    parser.add_argument("-d", "--directory", required=True, help="Input directory containing .ioc files")
    parser.add_argument("-o", "--output",
                        help="Output JSON file path (optional, defaults to input file name with .json)")

    args = parser.parse_args()
    input_dir = args.directory

    if not os.path.isdir(input_dir):
        print(f"Error: Directory '{input_dir}' not found.")
        exit(1)

    ioc_files = [f for f in os.listdir(input_dir) if f.endswith(".ioc")]

    if not ioc_files:
        print("Error: No .ioc files found in the specified directory.")
        exit(1)

    input_path = os.path.join(input_dir, ioc_files[0])
    print(f"Processing: {input_path}")

    parsed_data = parse_ioc_file(input_path)
    if parsed_data:
        output_file = args.output if args.output else os.path.splitext(input_path)[0] + ".json"
        save_to_json(parsed_data, output_file)
        print_summary(parsed_data)
        print(f"JSON output saved to: {output_file}")
