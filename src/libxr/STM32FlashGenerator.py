import re
from dataclasses import dataclass
from typing import Optional, List
import sys
import traceback
import yaml


@dataclass
class FlashSector:
    """Flash memory sector information"""
    index: int
    address: int
    size: int


@dataclass
class FlashInfo:
    """Complete flash memory configuration"""
    model: str
    flash_base: int
    flash_sectors: List[FlashSector]
    flash_size_kb: Optional[int] = None


FLASH_SIZE_CODES = {
    '4': 16, '6': 32, '8': 64, 'B': 128, 'C': 256, 'D': 384,
    'E': 512, 'F': 768, 'G': 1024, 'H': 1536, 'I': 2048,
    'J': 3072, 'K': 4096, 'L': 6144, 'M': 8192, 'N': 12288,
    'P': 16384, 'Q': 512, 'R': 640, 'S': 768, 'T': 1024, 'Z': 512,
    '5': 512, '7': 2048, 'A': 1024, 'V': 1024, 'W': 512, 'X': 1024, 'Y': 2048,
    '1': 256, '3': 8,
}


def get_flash_kb(model: str) -> int:
    """Get flash size from STM32 model number (original logic preserved)"""
    model = model.strip().upper()

    # Original condition chain preserved
    if model.startswith("STM32WL"):
        if "X" in model:
            return 1024
        if any(code in model for code in ['55', '54', '53', '52']):
            return 512
        return 512
    if model.startswith("STM32WBA"):
        code = model[11]
        return FLASH_SIZE_CODES.get(code, 512)
    if model.startswith("STM32WB"):
        if "7" in model:
            return 2048
        if "V" in model:
            return 1024
        return 512
    if model.startswith("STM32U5"):
        if "A" in model:
            return 1024
        code = model[10]
        return FLASH_SIZE_CODES.get(code, None)

    # Original fallback logic
    try:
        return FLASH_SIZE_CODES[model[10]]
    except (KeyError, IndexError):
        raise ValueError(f"Unrecognized capacity code for {model}")


def layout_flash(model: str) -> FlashInfo:
    """Flash layout generator (original logic fully preserved)"""
    model = model.upper()
    flash_kb = get_flash_kb(model)
    flash_base = 0x08000000
    layout = []

    # Original condition chain
    if "F1" in model:
        page_size = 1024 if flash_kb <= 128 else 2048
        layout = [((flash_kb * 1024) // page_size, page_size)]
    elif "F2" in model or "F4" in model:
        sector_sizes = [16, 16, 16, 16, 64] + [128] * 12
        layout = []
        remaining = flash_kb
        for size in sector_sizes:
            if remaining >= size:
                layout.append((1, size * 1024))
                remaining -= size
            else:
                break
    elif "U5" in model:
        layout = [(1, 128 * 1024)] * (flash_kb // 128)
    elif any(x in model for x in ["WL", "L4", "G0", "G4", "C0", "U0"]):
        layout = [((flash_kb * 1024) // 2048, 2048)]
    elif "L0" in model:
        layout = [((flash_kb * 1024) // 128, 128)]
    elif "WB" in model:
        layout = [((flash_kb * 1024) // 4096, 4096)]
    elif "WBA" in model:
        layout = [(1, 32 * 1024)] * (flash_kb // 32)
    elif "H7" in model:
        layout = [((flash_kb * 1024) // (128 * 1024), 128 * 1024)]
    elif any(x in model for x in ["F7", "F3", "F0"]):
        layout = [((flash_kb * 1024) // 2048, 2048)]
    elif "L1" in model:
        layout = [((flash_kb * 1024) // 256, 256)]
    else:
        layout = [((flash_kb * 1024) // 1024, 1024)]

    # Original sector generation
    sectors = []
    addr = flash_base
    for count, size in layout:
        for _ in range(count):
            sectors.append(FlashSector(
                index=len(sectors),
                address=addr,
                size=size
            ))
            addr += size

    return FlashInfo(
        model=model,
        flash_base=flash_base,
        flash_sectors=sectors,
        flash_size_kb=flash_kb
    )


def flash_info_to_dict(info: FlashInfo) -> dict:
    """Original serialization logic preserved"""
    return {
        "model": info.model,
        "flash_base": f"0x{info.flash_base:08X}",
        "flash_size_kb": info.flash_size_kb,
        "sectors": [
            {"index": s.index, "address": f"0x{s.address:08X}", "size_kb": round(s.size / 1024, 3)}
            for s in info.flash_sectors
        ]
    }


def main():
    from libxr.PackageInfo import LibXRPackageInfo

    LibXRPackageInfo.check_and_print()

    """Command line interface for STM32 flash information tool"""

    def validate_model(model: str) -> bool:
        """Validate STM32 model number format"""
        return (
                '-' not in model and
                model.upper().startswith("STM32") and
                len(model) > 8
        )

    if len(sys.argv) != 2:
        print("STM32 Flash Information Tool")
        print("Usage:")
        print(f"  xr_stm32_flash <STM32_MODEL>")
        print("\nExamples:")
        print(f"  xr_stm32_flash STM32F103C8T6")
        print(f"  xr_stm32_flash STM32L476RG")
        sys.exit(1)

    model = sys.argv[1].strip().upper()

    try:
        if not validate_model(model):
            raise ValueError(f"Invalid STM32 model format: {model}")

        info = layout_flash(model)
        print(yaml.safe_dump(
            flash_info_to_dict(info),
            sort_keys=False,
            allow_unicode=True,
            default_flow_style=False
        ))

    except Exception as e:
        print(f"\nERROR: Failed to process model {model}")
        print(f"Reason: {str(e)}")
        print("\nStack trace:")
        traceback.print_exc()
        sys.exit(2)


if __name__ == '__main__':
    main()