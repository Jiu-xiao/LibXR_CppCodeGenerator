from pathlib import Path

from libxr.PeripheralAnalyzerSTM32 import PeripheralParser, parse_ioc_file


def parse_ioc(tmp_path: Path, content: str) -> dict:
    ioc_path = tmp_path / "case.ioc"
    ioc_path.write_text(content, encoding="utf-8")
    parsed = parse_ioc_file(str(ioc_path))
    assert parsed is not None
    return parsed


def test_gpio_pin_aliases_are_normalized(tmp_path: Path) -> None:
    parsed = parse_ioc(
        tmp_path,
        """
Mcu.Family=STM32H7
Mcu.CPN=STM32H723VGTx
PC3_C.Signal=GPIO_Output
PC3_C.GPIO_Label=LED_C
PB2/BOOT1.Signal=GPIO_Input
PB2/BOOT1.GPIO_Label=BOOT1_IN
PA13(JTMS/SWDIO).Signal=SYS_JTMS-SWDIO
PA13(JTMS/SWDIO).GPIO_Label=
PH0\\ -\\ OSC_IN.Signal=RCC_OSC_IN
""",
    )

    assert parsed["GPIO"] == {
        "PC3": {"Signal": "GPIO_Output", "Label": "LED_C"},
        "PB2": {"Signal": "GPIO_Input", "Label": "BOOT1_IN"},
    }
    assert parsed["Mcu"] == {"Family": "STM32H7", "Type": "STM32H723VGTx"}


def test_signal_aliases_drive_timer_i2c_and_uart_detection(tmp_path: Path) -> None:
    parsed = parse_ioc(
        tmp_path,
        """
Mcu.IP0=I2C1
Mcu.IP1=USB_OTG_FS
PB6.Signal=I2C1_SCL
PB7.Signal=S_I2C1_SDA
PA8.Signal=S_TIM1_CH1
PA8.GPIO_Label=PWM1
TIM1.Channel=TIM_CHANNEL_1
PC10.Signal=USART3_TX
PC11.Signal=S_USART3_RX
USB_OTG_FS.IPParameters=VirtualMode
USB_OTG_FS.VirtualMode=Device_Only
""",
    )

    assert parsed["Peripherals"]["I2C"]["I2C1"]["Pins"] == {
        "SCL": "PB6",
        "SDA": "PB7",
    }
    assert parsed["Peripherals"]["TIM"]["TIM1"]["Channels"]["CH1"]["Label"] == "PWM1"
    assert parsed["Peripherals"]["USART"]["USART3"]["Mode"] == "Asynchronous"
    assert parsed["Peripherals"]["USB"]["USB_OTG_FS"]["IPParameters"] == ["VirtualMode"]


def test_dma_request_ids_and_config_keys_are_not_confused(tmp_path: Path) -> None:
    parsed = parse_ioc(
        tmp_path,
        """
USART3.BaudRate=115200
SPI6.BaudRate=1000000
Dma.Request0=USART3_RX
Dma.RequestsNb=1
Dma.USART3_RX.0.Instance=DMA1_Stream1
Dma.USART3_RX.0.Direction=DMA_PERIPH_TO_MEMORY
Dma.USART3_RX.0.PeriphDataAlignment=DMA_PDATAALIGN_BYTE
Dma.USART3_RX.0.MemDataAlignment=DMA_MDATAALIGN_BYTE
Dma.USART3_RX.0.Mode=DMA_NORMAL
Dma.USART3_RX.0.Priority=DMA_PRIORITY_VERY_HIGH
Bdma.Request0=SPI6_TX
Bdma.RequestsNb=1
Bdma.SPI6_TX.0.Instance=BDMA_Channel0
""",
    )

    assert parsed["DMA"]["Requests"] == {
        "Dma.Request0": "USART3_RX",
        "Bdma.Request0": "SPI6_TX",
    }
    assert "Dma.RequestsNb" not in parsed["DMA"]["Requests"]
    assert "Bdma.RequestsNb" not in parsed["DMA"]["Requests"]
    assert "USART3_RX_0" in parsed["DMA"]["Configurations"]
    assert "SPI6_TX_0" in parsed["DMA"]["Configurations"]
    assert "Request0" not in parsed["DMA"]["Configurations"]
    assert parsed["Peripherals"]["USART"]["USART3"]["DMA_RX"] == "ENABLE"
    assert "DMA_rx" not in parsed["Peripherals"]["USART"]["USART3"]
    assert parsed["Peripherals"]["SPI"]["SPI6"]["DMA_TX_TYPE"] == "BDMA"
    assert parsed["DMA"]["Configurations"]["USART3_RX_0"]["direction_full"] == "periph_to_memory"


def test_low_level_ioc_token_helpers() -> None:
    assert PeripheralParser._normalize_gpio_pin_token("PC3_C") == "PC3"
    assert PeripheralParser._normalize_gpio_pin_token("PB2/BOOT1") == "PB2"
    assert PeripheralParser._normalize_gpio_pin_token("PA13(JTMS/SWDIO)") == "PA13"
    assert PeripheralParser._normalize_signal_token("S_TIM1_CH1") == "TIM1_CH1"
    assert PeripheralParser._signal_root("S_LPUART1_TX") == "LPUART1"
    assert PeripheralParser._signal_suffix("S_I2C3_SDA") == "SDA"
    assert PeripheralParser._dma_request_id("Dma.Request12", "Dma") == "12"
    assert PeripheralParser._dma_request_id("Dma.RequestsNb", "Dma") is None
