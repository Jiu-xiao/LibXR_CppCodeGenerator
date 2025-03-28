#!/usr/bin/env python
"""STM32CubeMX IOC Configuration Parser - Optimized Version"""

import argparse
import os
import re
import logging
from typing import (
    Dict,
    List,
    Union,
    Optional,
    Pattern,
    DefaultDict,
    Any,
    TextIO,
    Match,
)
from collections import defaultdict
import yaml

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")


# --------------------------
# Utility Functions
# --------------------------
def sanitize_numeric(value: str) -> Union[int, float, str]:
    """Convert string to appropriate numeric type if possible."""
    try:
        return int(value) if value.isdigit() else float(value)
    except ValueError:
        return value


# --------------------------
# Configuration Containers
# --------------------------
class ConfigurationManager:
    """Centralized storage and processing of parsed configuration data."""

    def __init__(self) -> None:
        self.gpio_pins: DefaultDict[str, Dict[str, Any]] = defaultdict(dict)
        self.peripherals: DefaultDict[str, DefaultDict[str, Dict]] = defaultdict(
            lambda: defaultdict(dict)
        )
        self.dma_requests: Dict[str, str] = {}
        self.dma_configs: DefaultDict[str, List[Dict]] = defaultdict(list)
        self.freertos_config: Dict[str, Any] = {
            "Tasks": {},
            "Heap": None,
            "Features": {}
        }
        self.timebase: Dict[str, Optional[str]] = {"Source": "SysTick", "IRQ": None}
        self.mcu_config: Dict[str, Optional[str]] = {"Family": None, "Type": None}

    def clean_structure(self) -> Dict[str, Any]:
        """Apply data cleansing rules and return final structure."""
        cleaned_data = {
            "GPIO": self._clean_gpio(),
            "Peripherals": self._clean_peripherals(),
            "DMA": {
                "Requests": self.dma_requests,
                "Configurations": self._clean_dma_configs()
            },
            "Timebase": self.timebase,
            "Mcu": self.mcu_config
        }

        # Conditionally add FreeRTOS section
        cleaned_freertos = self._clean_freertos()
        if any([cleaned_freertos["Tasks"],
                cleaned_freertos["Heap"] is not None,
                cleaned_freertos["Features"]]):
            cleaned_data["FreeRTOS"] = cleaned_freertos

        return cleaned_data


    def _clean_gpio(self) -> Dict[str, Dict]:
        return {
            pin: config
            for pin, config in self.gpio_pins.items()
            if self._is_valid_gpio(config)
        }

    def _is_valid_gpio(self, config: Dict) -> bool:
        return config.get("Signal") in {"GPIO_Output", "GPIO_Input"} or \
            config.get("Signal", "").startswith("GPXTI")

    def _clean_peripherals(self) -> Dict[str, Dict]:
        return {
            p_type: {p: self._clean_peripheral_config(cfg)
                     for p, cfg in p_group.items()}
            for p_type, p_group in self.peripherals.items()
        }

    def _clean_peripheral_config(self, config: Dict) -> Dict:
        return {k: v for k, v in config.items() if v not in (None, "", [], {})}

    def _clean_dma_configs(self) -> Dict[str, List]:
        return {k: v for k, v in self.dma_configs.items() if v}

    def _clean_freertos(self) -> Dict:
        return {
            "Tasks": self.freertos_config["Tasks"],
            "Heap": self.freertos_config["Heap"],
            "Features": [
                feat.replace("INCLUDE_", "")
                for feat, enabled in self.freertos_config["Features"].items()
                if enabled
            ]
        }


# --------------------------
# Base Parser Class
# --------------------------
class PeripheralParser:
    """Abstract base class for peripheral-specific parsers."""

    def __init__(
            self,
            config: ConfigurationManager,
            raw_map: Dict[str, str],
            gpio_pattern: Pattern = re.compile(r"^(P[A-I]\d+(?:-[\w]+)*)\.(Signal|GPIO_Label|GPIO_PuPd)")
    ) -> None:
        self.config = config
        self.raw_map = raw_map
        self.gpio_pattern = gpio_pattern

    def parse_gpio(self) -> None:
        """Common GPIO parsing logic."""
        for key, value in self.raw_map.items():
            if match := self.gpio_pattern.match(key):
                pin, prop = match.groups()
                self._process_gpio_property(pin, prop, value)

    def _process_gpio_property(self, pin: str, prop: str, value: str) -> None:
        """Handle individual GPIO property."""
        prop_map = {
            "Signal": ("Signal", value),
            "GPIO_Label": ("Label", re.match(r"^\S+", value).group(0)),
            "GPIO_PuPd": ("Pull", value),
        }
        field, val = prop_map[prop]
        self.config.gpio_pins[pin][field] = val
        if "GPXTI" in value:
            self.config.gpio_pins[pin]["GPXTI"] = True

    def parse(self, p_type: str) -> None:
        """Template method to be implemented by subclasses."""
        raise NotImplementedError


# --------------------------
# MCU Parser
# --------------------------
class McuParser(PeripheralParser):
    """Handle MCU-related configurations."""

    def parse(self, p_type: str) -> None:
        for key, value in self.raw_map.items():
            if not key.startswith("Mcu"):
                continue
            parts = key.split(".")
            if "Family" in parts[1]:
                self.config.mcu_config["Family"] = value
            elif "CPN" in parts[1]:
                self.config.mcu_config["Type"] = value


# --------------------------
# TIM Parser
# --------------------------
class TIMParser(PeripheralParser):
    """Handle TIM (Timer) peripheral configurations."""

    def parse(self, p_type: str) -> None:
        for key, value in self.raw_map.items():
            if not key.startswith("TIM"):
                continue

            parts = key.split(".")
            tim_name = parts[0]
            self._ensure_tim_instance(p_type, tim_name)

            if "Channel-PWM" in key:
                self._handle_pwm_channel(tim_name, parts, value)
            elif "Period" in parts[1]:
                self.config.peripherals[p_type][tim_name]["Period"] = sanitize_numeric(value)
            elif "Prescaler" in parts[1]:
                self.config.peripherals[p_type][tim_name]["Prescaler"] = sanitize_numeric(value)
            elif "Mode" in parts[1]:
                self.config.peripherals[p_type][tim_name]["Mode"] = value

    def _ensure_tim_instance(self, p_type: str, tim_name: str) -> None:
        """Initialize TIM instance if not exists."""
        if not self.config.peripherals[p_type].get(tim_name):
            self.config.peripherals[p_type][tim_name] = {
                "Mode": None,
                "ClockPrescaler": None,
                "Period": None,
                "Prescaler": None,
                "Channels": {},
                "Pulses": {}
            }

    def _handle_pwm_channel(self, tim_name: str, parts: list, value: str) -> None:
        """Extract PWM channel configuration."""
        if ch_match := re.search(r"CH(\d+)", parts[1]):
            ch_num = ch_match.group(1)
            channel_id = f"CH{ch_num}"
            pin_label = self._get_associated_pin_label(parts[0])

            self.config.peripherals["TIM"][tim_name]["Channels"][channel_id] = {
                "Label": pin_label,
                "PWM": True,
                "DutyCycle": sanitize_numeric(value) if value.isdigit() else None
            }

    def _get_associated_pin_label(self, timer_pin: str) -> str:
        """Retrieve GPIO label from pin configuration."""
        return self.config.gpio_pins.get(timer_pin, {}).get("Label", timer_pin)


# --------------------------
# ADC Parser
# --------------------------
class ADCParser(PeripheralParser):
    """Parse ADC configurations with strict channel validation."""

    # Regular expression to match valid ADC channels (e.g. ADC_CHANNEL_0)
    _CHANNEL_PATTERN = re.compile(r"^ADC_CHANNEL_[\w]+$")

    def parse(self, p_type: str) -> None:
        """Process ADC configurations with enhanced validation."""
        for key, value in self.raw_map.items():
            if not key.startswith("ADC"):
                continue

            parts = key.split(".")
            adc_name = parts[0]
            self._ensure_adc_instance(adc_name)

            if "ChannelRegularConversion" in key:
                self._process_conversion_entry(adc_name, value)
            elif "ContinuousConvMode" in parts[1]:
                self.config.peripherals["ADC"][adc_name]["ContinuousMode"] = value == "ENABLE"
            elif "DMARegular" in parts[1]:
                self.config.peripherals["ADC"][adc_name]["DMA"] = value

    def _ensure_adc_instance(self, adc_name: str) -> None:
        """Initialize ADC instance with cleaned structure."""
        if adc_name not in self.config.peripherals["ADC"]:
            self.config.peripherals["ADC"][adc_name] = {
                "ContinuousMode": False,
                "RegularConversions": [],
                "Channels": [],
                "DMA": "DISABLE"
            }

    def _process_conversion_entry(self, adc_name: str, raw_value: str) -> None:
        """Extract and validate ADC channel entries."""
        # Split and process all comma-separated values
        for entry in raw_value.split(","):
            cleaned_entry = entry.strip()

            # Validate entry format using regex
            if self._is_valid_channel(cleaned_entry):
                self._add_unique_entry(adc_name, "Channels", cleaned_entry)
                self._add_unique_entry(adc_name, "RegularConversions", cleaned_entry)
            elif cleaned_entry:  # Log non-empty invalid entries
                logging.debug(f"Ignored invalid ADC entry: {cleaned_entry}")

    def _is_valid_channel(self, entry: str) -> bool:
        """Validate channel name format using regex pattern."""
        return bool(self._CHANNEL_PATTERN.match(entry))

    def _add_unique_entry(self, adc_name: str, field: str, value: str) -> None:
        """Add value to list only if not already present."""
        target_list = self.config.peripherals["ADC"][adc_name][field]
        if value not in target_list:
            target_list.append(value)


# --------------------------
# SPI Parser
# --------------------------
class SPIParser(PeripheralParser):
    """Handle SPI peripheral configurations."""

    def parse(self, p_type: str) -> None:
        for key, value in self.raw_map.items():
            if not key.startswith("SPI"):
                continue

            parts = key.split(".")
            spi_name = parts[0]
            self._ensure_spi_instance(p_type, spi_name)

            prop = parts[1]
            if "BaudRate" in prop:
                self.config.peripherals[p_type][spi_name]["BaudRate"] = sanitize_numeric(value)
            elif "Direction" in prop:
                self.config.peripherals[p_type][spi_name]["Direction"] = value
            elif "CLKPolarity" in prop:
                self.config.peripherals[p_type][spi_name]["CLKPolarity"] = value
            elif "CLKPhase" in prop:
                self.config.peripherals[p_type][spi_name]["CLKPhase"] = value

    def _ensure_spi_instance(self, p_type: str, spi_name: str) -> None:
        """Initialize SPI instance if not exists."""
        if not self.config.peripherals[p_type].get(spi_name):
            self.config.peripherals[p_type][spi_name] = {
                "BaudRate": None,
                "Direction": None,
                "CLKPolarity": None,
                "CLKPhase": None,
                "DMA": {}
            }


# --------------------------
# USART/UART Parser
# --------------------------
class USARTParser(PeripheralParser):
    """Handle USART/UART peripheral configurations."""

    def parse(self, p_type: str) -> None:
        for key, value in self.raw_map.items():
            if not key.startswith(("USART", "UART", "LPUART")):
                continue

            parts = key.split(".")
            uart_name = parts[0]
            self._ensure_uart_instance(p_type, uart_name)

            prop = parts[1]
            if "BaudRate" in prop:
                self.config.peripherals[p_type][uart_name]["BaudRate"] = sanitize_numeric(value)
            elif "WordLength" in prop:
                self.config.peripherals[p_type][uart_name]["WordLength"] = value
            elif "Parity" in prop:
                self.config.peripherals[p_type][uart_name]["Parity"] = value
            elif "StopBits" in prop:
                self.config.peripherals[p_type][uart_name]["StopBits"] = value
            elif "Mode" in prop:
                self._handle_operation_mode(uart_name, value)

    def _ensure_uart_instance(self, p_type: str, uart_name: str) -> None:
        """Initialize UART instance if not exists."""
        if not self.config.peripherals[p_type].get(uart_name):
            self.config.peripherals[p_type][uart_name] = {
                "BaudRate": None,
                "WordLength": None,
                "Parity": None,
                "StopBits": None,
                "Mode": "Asynchronous",
                "DMA": {}
            }

    def _handle_operation_mode(self, uart_name: str, value: str) -> None:
        """Decode complex mode configurations."""
        if "IrDA" in value:
            self.config.peripherals["USART"][uart_name]["Mode"] = "IrDA"
        elif "LIN" in value:
            self.config.peripherals["USART"][uart_name]["Mode"] = "LIN"
        elif "SmartCard" in value:
            self.config.peripherals["USART"][uart_name]["Mode"] = "SmartCard"


# --------------------------
# I2C Parser
# --------------------------
class I2CParser(PeripheralParser):
    """Handle I2C peripheral configurations."""

    def parse(self, p_type: str) -> None:
        for key, value in self.raw_map.items():
            if not key.startswith("I2C"):
                continue

            parts = key.split(".")
            i2c_name = parts[0]
            self._ensure_i2c_instance(p_type, i2c_name)

            prop = parts[1]
            if "ClockSpeed" in prop:
                self.config.peripherals[p_type][i2c_name]["ClockSpeed"] = sanitize_numeric(value)
            elif "DutyCycle" in prop:
                self.config.peripherals[p_type][i2c_name]["DutyCycle"] = value
            elif "AddressingMode" in prop:
                self.config.peripherals[p_type][i2c_name]["AddressingMode"] = value
            elif "DualAddressMode" in prop:
                self.config.peripherals[p_type][i2c_name]["DualAddressMode"] = value == "ENABLE"

    def _ensure_i2c_instance(self, p_type: str, i2c_name: str) -> None:
        """Initialize I2C instance if not exists."""
        if not self.config.peripherals[p_type].get(i2c_name):
            self.config.peripherals[p_type][i2c_name] = {
                "ClockSpeed": None,
                "DutyCycle": None,
                "AddressingMode": "7-bit",
                "DualAddressMode": False,
                "NoStretchMode": False,
                "DMA": {}
            }


# --------------------------
# CAN/FDCAN Parser
# --------------------------
class CANParser(PeripheralParser):
    """Handle both CAN and FDCAN peripheral configurations."""

    def parse(self, p_type: str) -> None:
        """Process CAN/FDCAN parameters with legacy support."""
        for key, value in self.raw_map.items():
            if not key.startswith(("CAN", "FDCAN")):
                continue

            parts = key.split(".")
            can_name = parts[0]
            p_type = "FDCAN" if can_name.startswith("FDCAN") else "CAN"

            self._ensure_can_instance(p_type, can_name)
            prop = parts[1]

            # Common parameters
            if "CalculateBaudRate" in prop:
                self.config.peripherals[p_type][can_name]["BaudRate"] = value
            elif "Mode" in prop:
                self.config.peripherals[p_type][can_name]["Mode"] = value

            # CAN-specific parameters
            if p_type == "CAN":
                self._handle_legacy_can_params(can_name, prop, value)

            # FDCAN-specific parameters
            if p_type == "FDCAN":
                self._handle_fdcan_params(can_name, prop, value)

    def _ensure_can_instance(self, p_type: str, can_name: str) -> None:
        """Initialize CAN/FDCAN instance with proper structure."""
        if can_name not in self.config.peripherals[p_type]:
            defaults = {
                "CAN": {
                    "BaudRate": None,
                    "Mode": None,
                    "TimeSeg1": None,
                    "TimeSeg2": None,
                    "AutoRetransmission": False,
                    "AutoWakeup": False
                },
                "FDCAN": {
                    "NominalBaudRate": None,
                    "DataBaudRate": None,
                    "FrameFormat": None,
                    "StdFilters": 0,
                    "ExtFilters": 0
                }
            }
            self.config.peripherals[p_type][can_name] = defaults[p_type].copy()

    def _handle_legacy_can_params(self, can_name: str, prop: str, value: str) -> None:
        """Process legacy CAN 2.0 parameters."""
        param_map = {
            "BS1": "TimeSeg1",
            "BS2": "TimeSeg2",
            "ABOM": ("AutoRetransmission", lambda v: v == "ENABLE"),
            "AWUM": ("AutoWakeup", lambda v: v == "ENABLE")
        }

        if mapping := param_map.get(prop):
            if isinstance(mapping, tuple):
                key, converter = mapping
                self.config.peripherals["CAN"][can_name][key] = converter(value)
            else:
                self.config.peripherals["CAN"][can_name][mapping] = value

    def _handle_fdcan_params(self, can_name: str, prop: str, value: str) -> None:
        """Process FDCAN specific parameters."""
        param_map = {
            "NominalPrescaler": ("NominalBaudRate", float),
            "DataPrescaler": ("DataBaudRate", float),
            "FrameFormat": ("FrameFormat", str),
            "StdFiltersNbr": ("StdFilters", int),
            "ExtFiltersNbr": ("ExtFilters", int)
        }

        if mapping := param_map.get(prop):
            key, converter = mapping
            try:
                self.config.peripherals["FDCAN"][can_name][key] = converter(value)
            except ValueError:
                logging.warning(f"Invalid {key} value for {can_name}: {value}")


# --------------------------
# USB Parser
# --------------------------
class USBParser(PeripheralParser):
    """Handle USB peripheral configurations with endpoint parsing."""

    _ENDPOINT_PATTERN = re.compile(r"EP(\d+)_(\w+)")  # EP1_Mode, EP2_Type etc.

    def parse(self, p_type: str) -> None:
        """Process USB parameters and endpoint configurations."""
        for key, value in self.raw_map.items():
            if not key.startswith("USB"):
                continue

            parts = key.split(".")
            usb_name = parts[0]
            self._ensure_usb_instance(usb_name)

            # Global USB properties
            if len(parts) == 2:
                self._process_global_property(usb_name, parts[1], value)

            # Endpoint configurations (EP1_Mode, EP2_Type etc.)
            elif len(parts) > 2 and "EP" in parts[1]:
                self._process_endpoint(usb_name, parts[1], value)

    def _ensure_usb_instance(self, usb_name: str) -> None:
        """Initialize USB instance with default structure."""
        if usb_name not in self.config.peripherals["USB"]:
            self.config.peripherals["USB"][usb_name] = {
                "Mode": None,
                "Speed": None,
                "VBus": False,
                "Endpoints": defaultdict(dict)
            }

    def _process_global_property(self, usb_name: str, prop: str, value: str) -> None:
        """Handle global USB properties."""
        prop_map = {
            "Mode": ("Mode", str),
            "Speed": ("Speed", str),
            "VbusMonitoring": ("VBus", lambda v: v == "ENABLE")
        }

        if mapping := prop_map.get(prop):
            key, converter = mapping
            try:
                self.config.peripherals["USB"][usb_name][key] = converter(value)
            except ValueError:
                logging.warning(f"Invalid USB {prop} value: {value}")

    def _process_endpoint(self, usb_name: str, ep_key: str, value: str) -> None:
        """Parse endpoint configurations with validation."""
        if match := self._ENDPOINT_PATTERN.match(ep_key):
            ep_num = match.group(1)
            ep_prop = match.group(2)
            endpoint_id = f"EP{ep_num}"

            # Validate endpoint number (0-15 for USB FS)
            if 0 <= int(ep_num) <= 15:
                endpoint = self.config.peripherals["USB"][usb_name]["Endpoints"][endpoint_id]

                # Special handling for endpoint type
                if ep_prop == "Type":
                    endpoint["Type"] = self._normalize_ep_type(value)
                else:
                    endpoint[ep_prop] = value

    def _normalize_ep_type(self, raw_type: str) -> str:
        """Convert CubeMX EP types to simplified format."""
        type_map = {
            "BULK": "Bulk",
            "INTERRUPT": "Interrupt",
            "ISOCHRONOUS": "Isochronous",
            "CONTROL": "Control"
        }
        return type_map.get(raw_type.split("_")[-1], "Unknown")


# --------------------------
# DMA Parser
# --------------------------
class DMAParser(PeripheralParser):
    """Parses and structures DMA configurations from .ioc files"""

    # Enhanced property mapping: CubeMX param → friendly name
    _PROPERTY_MAP = {
        "Instance": ("stream", str),
        "Direction": ("direction", lambda v: v.split("_")[-1]),
        "PeriphInc": ("periph_inc", lambda v: v == "ENABLE"),
        "MemInc": ("mem_inc", lambda v: v == "ENABLE"),
        "PeriphDataAlignment": ("periph_align", lambda v: v.split("_")[-1].lower()),
        "MemDataAlignment": ("mem_align", lambda v: v.split("_")[-1].lower()),
        "Mode": ("mode", lambda v: v.split("_")[-1].capitalize()),
        "Priority": ("priority", lambda v: v.split("_")[-1].replace("VERY", "").strip().capitalize()),
        "FIFOMode": ("fifo", lambda v: "Enabled" if "ENABLE" in v else "Disabled")
    }

    def parse(self, p_type: str) -> None:
        """Three-phase parsing workflow"""
        self._parse_requests()
        self._parse_configs()
        self._link_configs()

    def _parse_requests(self) -> None:
        """Extract DMA request mapping (RequestID → Peripheral)"""
        for key, value in self.raw_map.items():
            if key.startswith("Dma.Request"):
                # Handle keys like: Dma.Request11=ADC1
                req_id = key.split("Request")[1].split("=")[0].strip()
                self.config.dma_requests[req_id] = value

    def _parse_configs(self) -> None:
        """Convert raw DMA configs to structured format"""
        config_map = defaultdict(dict)

        # Phase 1: Group raw properties by config key
        for key, value in self.raw_map.items():
            if not key.startswith("Dma.") or key.count(".") < 2:
                continue

            # Parse key structure: Dma.ADC1.11.Direction → (peripheral, req_id, prop)
            parts = key.split(".")
            peripheral = parts[1]
            req_id = parts[2]
            prop = parts[3] if len(parts) > 3 else "Instance"

            config_key = f"{peripheral}_{req_id}"
            config_map[config_key][prop] = value
            config_map[config_key]["_request_id"] = req_id

        # Phase 2: Convert to structured format
        for config_key, props in config_map.items():
            structured = {
                "request_id": props.get("_request_id", ""),
                "peripheral": self.config.dma_requests.get(props.get("_request_id", ""), "Unknown"),
                "stream": props.get("Instance", "")
            }

            # Property conversion
            for cube_prop, (field, converter) in self._PROPERTY_MAP.items():
                if cube_prop in props:
                    try:
                        structured[field] = converter(props[cube_prop])
                    except Exception as e:
                        logging.warning(f"DMA property conversion failed for {config_key}.{cube_prop}: {str(e)}")

            self.config.dma_configs[config_key] = structured

    def _link_configs(self) -> None:
        """Link DMA configs to corresponding peripherals"""
        for config_key, cfg in self.config.dma_configs.items():
            # Extract direction from peripheral name (e.g. SPI1_RX → SPI1, RX)
            peripheral_full = cfg["peripheral"]
            if "_" in peripheral_full:
                p_name, direction = peripheral_full.split("_", 1)
                direction = direction.lower()
            else:
                p_name = peripheral_full
                direction = "general"

            # Find matching peripheral
            for p_type in ["SPI", "I2C", "USART", "ADC", "TIM"]:
                if p_name in self.config.peripherals.get(p_type, {}):
                    dir_key = f"dma_{direction}" if direction != "general" else "dma"

                    # Initialize DMA structure if needed
                    if "dma" not in self.config.peripherals[p_type][p_name]:
                        self.config.peripherals[p_type][p_name]["dma"] = {}

                    # Store configuration
                    self.config.peripherals[p_type][p_name]["dma"][dir_key] = cfg
                    break


# --------------------------
# FreeRTOS Parser
# --------------------------
class FreeRTOSParser(PeripheralParser):
    """Handle FreeRTOS-related configurations."""

    def parse(self, p_type: str) -> None:
        for key, value in self.raw_map.items():
            if not key.startswith("FREERTOS"):
                continue

            parts = key.split(".")
            if parts[1].startswith("Tasks"):
                self._process_task_configuration(value)
            elif "HeapSize" in key:
                self.config.freertos_config["Heap"] = f"{sanitize_numeric(value)}B"
            elif "INCLUDE_" in key:
                self._process_feature_flag(parts[1], value)

    def _process_task_configuration(self, task_data: str) -> None:
        """Parse FreeRTOS task definitions."""
        elements = [x for x in task_data.split(",") if x and x != "NULL"]
        if len(elements) >= 5:
            task_name = elements[0]
            self.config.freertos_config["Tasks"][task_name] = {
                "Priority": elements[1],
                "StackSize": f"{elements[2]}B",
                "EntryFunction": elements[3],
                "Type": elements[4]
            }

    def _process_feature_flag(self, feature: str, state: str) -> None:
        """Track enabled FreeRTOS features."""
        self.config.freertos_config["Features"][feature] = state == "ENABLE"


# --------------------------
# Core Parsing Workflow
# --------------------------
def parse_ioc_file(ioc_path: str) -> Optional[Dict[str, Any]]:
    """Orchestrate the parsing of an .ioc file through registered parsers."""
    config = ConfigurationManager()

    try:
        with open(ioc_path, "r", encoding="utf-8") as f:
            raw_map = _extract_key_value_pairs(f)
    except (UnicodeDecodeError, IOError) as e:
        logging.error(f"File processing failed: {str(e)}")
        return None

    # Instantiate all parsers
    parsers = [
        FreeRTOSParser(config, raw_map),
        McuParser(config, raw_map),
        TIMParser(config, raw_map),
        ADCParser(config, raw_map),
        SPIParser(config, raw_map),
        USARTParser(config, raw_map),
        I2CParser(config, raw_map),
        CANParser(config, raw_map),
        USBParser(config, raw_map),
        DMAParser(config, raw_map),
    ]

    # Execute parsing workflow
    try:
        # Phase 1: Common GPIO parsing
        parsers[0].parse_gpio()  # All parsers inherit GPIO capability

        # Phase 2: Peripheral-specific parsing
        for parser in parsers:
            if isinstance(parser, McuParser):
                parser.parse("Mcu")
            else:
                parser.parse(parser.__class__.__name__[:-6])  # Strip 'Parser' suffix

        # Phase 3: Post-processing
        _link_dma_requests(config)

        return config.clean_structure()
    except Exception as e:
        logging.error(f"Parsing failed: {str(e)}")
        return None


def _extract_key_value_pairs(file_handler: TextIO) -> Dict[str, str]:
    """Robust key-value extraction with line validation."""
    raw_map = {}
    for line_num, line in enumerate(file_handler, 1):
        line = line.strip()
        if not line or line.startswith("#"):
            continue

        try:
            key, value = map(str.strip, line.split("=", 1))
            raw_map[key.replace("\\#", "")] = value
        except ValueError:
            logging.warning(f"Ignored malformed entry at line {line_num}: {line}")

    return raw_map


def _link_dma_requests(config: ConfigurationManager) -> None:
    """Associate DMA requests with corresponding peripherals."""
    for req_id, peripheral in config.dma_requests.items():
        if "_" in peripheral:
            p_name, direction = peripheral.rsplit("_", 1)
            direction_key = f"DMA_{direction}"
        else:
            p_name = peripheral
            direction_key = "DMA"

        for p_type in ["USART", "SPI", "ADC", "I2C"]:
            if p_name in config.peripherals[p_type]:
                config.peripherals[p_type][p_name][direction_key] = "ENABLE"


# --------------------------
# Output Generation
# --------------------------
def save_to_yaml(data: Dict[str, Any], output_path: str = "parsed_ioc.yaml") -> bool:
    """Serialize configuration data to YAML with error handling."""
    try:
        with open(output_path, "w", encoding="utf-8") as f:
            yaml.dump(
                data,
                f,
                allow_unicode=True,
                sort_keys=False,
                default_flow_style=False,
                indent=2
            )
        logging.info(f"Configuration exported to: {output_path}")
        return True
    except (IOError, yaml.YAMLError) as e:
        logging.error(f"YAML export failed: {str(e)}")
        return False


def print_summary(data: Dict[str, Any]) -> None:
    """Generate human-readable configuration summary."""
    print("\n===== [Configuration Summary] =====")

    # MCU Info
    mcu = data.get("Mcu", {})
    print(f"\nMCU: {mcu.get('Family', 'Unknown')} {mcu.get('Type', '')}")

    # GPIO Summary
    gpio = data.get("GPIO", {})
    print(f"\nGPIO ({len(gpio)} pins):")
    print(f"  Outputs: {sum(1 for c in gpio.values() if c.get('Signal') == 'GPIO_Output')}")
    print(f"  Inputs: {sum(1 for c in gpio.values() if c.get('Signal') == 'GPIO_Input')}")
    print(f"  External Interrupts: {sum(1 for c in gpio.values() if c.get('GPXTI'))}")

    # Peripheral Summary
    print("\nActive Peripherals:")
    for p_type, group in data.get("Peripherals", {}).items():
        print(f"  {p_type}: {len(group)} instance(s)")
        for name, cfg in group.items():
            print(f"    {name}: {_format_peripheral_config(p_type, cfg)}")


def _format_peripheral_config(p_type: str, config: Dict) -> str:
    """Generate single-line peripheral configuration summary."""
    if p_type == "TIM":
        return f"Mode={config.get('Mode')} | Period={config.get('Period')}"
    elif p_type == "ADC":
        return f"Channels={len(config.get('RegularConversions', []))}"
    elif p_type in ["SPI", "I2C", "USART"]:
        return f"Baud={config.get('BaudRate') or config.get('ClockSpeed')}"
    return ""


# --------------------------
# Main Entry Point
# --------------------------
def main() -> None:
    """Command line interface handler."""
    parser = argparse.ArgumentParser(
        description="STM32CubeMX IOC Configuration Parser v2.0",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "-d", "--directory",
        required=True,
        help="Input directory containing .ioc files"
    )
    parser.add_argument(
        "-o", "--output",
        help="Custom output YAML file path (default: <input_file>.yaml)"
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Enable debug logging"
    )

    args = parser.parse_args()

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    if not os.path.isdir(args.directory):
        logging.error(f"Invalid input directory: {args.directory}")
        exit(1)

    ioc_files = [f for f in os.listdir(args.directory) if f.endswith(".ioc")]
    if not ioc_files:
        logging.error("No .ioc files found in target directory")
        exit(1)

    for ioc_file in ioc_files:
        input_path = os.path.join(args.directory, ioc_file)
        logging.info(f"Processing {ioc_file}...")

        config_data = parse_ioc_file(input_path)
        if not config_data:
            continue

        output_path = args.output or os.path.splitext(input_path)[0] + ".yaml"
        if save_to_yaml(config_data, output_path):
            print_summary(config_data)


if __name__ == "__main__":
    main()
