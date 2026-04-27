import re
import sys
import traceback
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
from typing import List, Optional, Pattern, Tuple

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


@dataclass(frozen=True)
class FlashRule:
    name: str
    prefix: Optional[str]
    contains: Optional[str]
    regex: Optional[Pattern[str]]
    flash_kb: Optional[int]
    flash_kb_min: Optional[int]
    flash_kb_max: Optional[int]
    layout_type: str
    size_bytes: Optional[int]
    pattern_kb: Tuple[int, ...]
    bank_kb: Optional[int]
    bank_address_stride_kb: Optional[int]


FLASH_SIZE_CODES = {
    "4": 16,
    "6": 32,
    "8": 64,
    "B": 128,
    "C": 256,
    "D": 384,
    "E": 512,
    "F": 768,
    "G": 1024,
    "H": 1536,
    "I": 2048,
    "J": 3072,
    "K": 4096,
    "L": 6144,
    "M": 8192,
    "N": 12288,
    "P": 16384,
    "Q": 512,
    "R": 640,
    "S": 768,
    "T": 1024,
    "Z": 512,
    "5": 512,
    "7": 2048,
    "A": 1024,
    "V": 1024,
    "W": 512,
    "X": 1024,
    "Y": 2048,
    "1": 256,
    "3": 8,
}

U5_FLASH_SIZE_CODES = dict(FLASH_SIZE_CODES)
U5_FLASH_SIZE_CODES["J"] = 4096

_FLASH_BASE = 0x08000000


def get_flash_kb(model: str) -> int:
    """Get flash size from STM32 model number (original logic preserved)"""
    model = model.strip().upper()

    if model.startswith("STM32N6"):
        raise ValueError("STM32N6 devices do not provide internal user flash")
    if model.startswith("STM32WL"):
        if "X" in model:
            return 1024
        if any(code in model for code in ["55", "54", "53", "52"]):
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
        code = model[10]
        return U5_FLASH_SIZE_CODES.get(code, None)

    try:
        return FLASH_SIZE_CODES[model[10]]
    except (KeyError, IndexError):
        raise ValueError(f"Unrecognized capacity code for {model}")


def layout_flash(model: str) -> FlashInfo:
    """Resolve STM32 flash layout from a single XML rule source."""
    model = model.strip().upper()
    flash_kb = get_flash_kb(model)

    rule = _match_flash_rule(model, flash_kb)
    if rule is None:
        sector_entries = _build_contiguous_sector_entries(
            _build_uniform_sector_sizes(flash_kb, 1024)
        )
    else:
        sector_entries = _build_sector_entries_from_rule(rule, flash_kb)

    return _build_flash_info(model, flash_kb, sector_entries)


def flash_info_to_dict(info: FlashInfo) -> dict:
    return {
        "model": info.model,
        "flash_base": f"0x{info.flash_base:08X}",
        "flash_size_kb": info.flash_size_kb,
        "sectors": [
            {
                "index": sector.index,
                "address": f"0x{sector.address:08X}",
                "size_kb": round(sector.size / 1024, 3),
            }
            for sector in info.flash_sectors
        ],
    }


def _build_flash_info(
    model: str,
    flash_kb: int,
    sector_entries: List[Tuple[int, int]],
) -> FlashInfo:
    sectors = []

    for address, size in sector_entries:
        sectors.append(
            FlashSector(
                index=len(sectors),
                address=address,
                size=size,
            )
        )

    return FlashInfo(
        model=model,
        flash_base=_FLASH_BASE,
        flash_sectors=sectors,
        flash_size_kb=flash_kb,
    )


def _build_contiguous_sector_entries(
    sector_sizes: List[int],
    start_address: int = _FLASH_BASE,
) -> List[Tuple[int, int]]:
    sector_entries = []
    address = start_address

    for size in sector_sizes:
        sector_entries.append((address, size))
        address += size

    return sector_entries


def _build_uniform_sector_sizes(flash_kb: int, size_bytes: int) -> List[int]:
    total_bytes = flash_kb * 1024
    if total_bytes % size_bytes != 0:
        raise ValueError(
            f"Flash size {flash_kb}KB is not divisible by erase size {size_bytes}B"
        )
    return [size_bytes] * (total_bytes // size_bytes)


def _build_sequence_sector_sizes(flash_kb: int, pattern_kb: Tuple[int, ...]) -> List[int]:
    remaining_kb = flash_kb
    sector_sizes = []

    for size_kb in pattern_kb:
        if remaining_kb < size_kb:
            break
        sector_sizes.append(size_kb * 1024)
        remaining_kb -= size_kb

    if remaining_kb != 0:
        raise ValueError(
            f"Flash size {flash_kb}KB is not fully covered by sequence {pattern_kb}"
        )
    return sector_sizes


def _build_banked_sector_entries(
    flash_kb: int,
    bank_kb: int,
    pattern_kb: Tuple[int, ...],
    bank_address_stride_kb: int,
) -> List[Tuple[int, int]]:
    if sum(pattern_kb) != bank_kb:
        raise ValueError(
            f"Bank pattern {pattern_kb} does not sum to bank size {bank_kb}KB"
        )

    remaining_kb = flash_kb
    bank_index = 0
    sector_entries = []

    while remaining_kb > 0:
        bank_remaining_kb = min(remaining_kb, bank_kb)
        bank_used_kb = 0
        address = _FLASH_BASE + bank_index * bank_address_stride_kb * 1024

        for size_kb in pattern_kb:
            if bank_used_kb + size_kb > bank_remaining_kb:
                break
            size_bytes = size_kb * 1024
            sector_entries.append((address, size_bytes))
            address += size_bytes
            bank_used_kb += size_kb

        if bank_used_kb != bank_remaining_kb:
            raise ValueError(
                f"Bank pattern {pattern_kb} does not cover {bank_remaining_kb}KB"
            )

        remaining_kb -= bank_remaining_kb
        bank_index += 1

    return sector_entries


def _build_sector_entries_from_rule(
    rule: FlashRule,
    flash_kb: int,
) -> List[Tuple[int, int]]:
    if rule.layout_type == "uniform":
        if rule.size_bytes is None:
            raise ValueError(f"Rule {rule.name} is missing size_bytes")
        return _build_contiguous_sector_entries(
            _build_uniform_sector_sizes(flash_kb, rule.size_bytes)
        )

    if rule.layout_type == "sequence":
        return _build_contiguous_sector_entries(
            _build_sequence_sector_sizes(flash_kb, rule.pattern_kb)
        )

    if rule.layout_type == "banked":
        if rule.bank_kb is None:
            raise ValueError(f"Rule {rule.name} is missing bank_kb")
        bank_address_stride_kb = rule.bank_address_stride_kb or rule.bank_kb
        return _build_banked_sector_entries(
            flash_kb,
            rule.bank_kb,
            rule.pattern_kb,
            bank_address_stride_kb,
        )

    raise ValueError(f"Unsupported flash layout type: {rule.layout_type}")


@lru_cache(maxsize=1)
def _load_flash_rules() -> Tuple[FlashRule, ...]:
    rule_path = Path(__file__).resolve().with_name("STM32FlashLayoutRules.xml")
    root = ET.parse(rule_path).getroot()
    rules = []

    for rule_elem in root.findall("rule"):
        child_elements = list(rule_elem)
        if len(child_elements) != 1:
            raise ValueError("Each flash rule must contain exactly one layout element")

        layout_elem = child_elements[0]
        layout_tag = _local_name(layout_elem.tag)
        size_bytes = None
        pattern_kb: Tuple[int, ...] = tuple()
        bank_kb = None

        if layout_tag == "uniform":
            size_bytes = _parse_rule_size_bytes(layout_elem)
        elif layout_tag == "sequence":
            pattern_kb = _parse_pattern_kb(layout_elem.attrib["sizes_kb"])
        elif layout_tag == "banked":
            bank_kb = int(layout_elem.attrib["bank_kb"], 0)
            pattern_kb = _parse_pattern_kb(layout_elem.attrib["sizes_kb"])
        else:
            raise ValueError(f"Unsupported layout tag: {layout_tag}")

        regex = rule_elem.attrib.get("regex")
        rules.append(
            FlashRule(
                name=rule_elem.attrib["name"],
                prefix=_normalize_optional(rule_elem.attrib.get("prefix")),
                contains=_normalize_optional(rule_elem.attrib.get("contains")),
                regex=re.compile(regex) if regex else None,
                flash_kb=_parse_optional_int(rule_elem.attrib.get("flash_kb")),
                flash_kb_min=_parse_optional_int(rule_elem.attrib.get("flash_kb_min")),
                flash_kb_max=_parse_optional_int(rule_elem.attrib.get("flash_kb_max")),
                layout_type=layout_tag,
                size_bytes=size_bytes,
                pattern_kb=pattern_kb,
                bank_kb=bank_kb,
                bank_address_stride_kb=_parse_optional_int(
                    layout_elem.attrib.get("address_stride_kb")
                ),
            )
        )

    return tuple(rules)


def _match_flash_rule(model: str, flash_kb: int) -> Optional[FlashRule]:
    for rule in _load_flash_rules():
        if rule.prefix is not None and not model.startswith(rule.prefix):
            continue
        if rule.contains is not None and rule.contains not in model:
            continue
        if rule.regex is not None and not rule.regex.search(model):
            continue
        if rule.flash_kb is not None and flash_kb != rule.flash_kb:
            continue
        if rule.flash_kb_min is not None and flash_kb < rule.flash_kb_min:
            continue
        if rule.flash_kb_max is not None and flash_kb > rule.flash_kb_max:
            continue
        return rule
    return None


def _parse_rule_size_bytes(elem: ET.Element) -> int:
    if "size_bytes" in elem.attrib:
        return int(elem.attrib["size_bytes"], 0)
    if "size_kb" in elem.attrib:
        return int(elem.attrib["size_kb"], 0) * 1024
    raise ValueError("uniform layout must provide size_bytes or size_kb")


def _parse_pattern_kb(spec: str) -> Tuple[int, ...]:
    pattern = []
    for token in spec.replace(" ", "").split(","):
        if not token:
            continue

        if "x" in token.lower():
            value_text, count_text = re.split(r"[xX]", token, maxsplit=1)
            value_kb = int(value_text, 0)
            count = int(count_text, 0)
            pattern.extend([value_kb] * count)
        else:
            pattern.append(int(token, 0))

    if not pattern:
        raise ValueError(f"Invalid pattern specification: {spec}")
    return tuple(pattern)


def _normalize_optional(value: Optional[str]) -> Optional[str]:
    return value.upper() if value else None


def _parse_optional_int(value: Optional[str]) -> Optional[int]:
    return int(value, 0) if value is not None else None


def _local_name(tag: str) -> str:
    return tag.split("}", 1)[-1]


def main():
    from libxr.PackageInfo import LibXRPackageInfo

    LibXRPackageInfo.check_and_print()

    def validate_model(model: str) -> bool:
        return "-" not in model and model.upper().startswith("STM32") and len(model) > 8

    if len(sys.argv) != 2:
        print("STM32 Flash Information Tool")
        print("Usage:")
        print("  xr_stm32_flash <STM32_MODEL>")
        print("\nExamples:")
        print("  xr_stm32_flash STM32F103C8T6")
        print("  xr_stm32_flash STM32L476RG")
        sys.exit(1)

    model = sys.argv[1].strip().upper()

    try:
        if not validate_model(model):
            raise ValueError(f"Invalid STM32 model format: {model}")

        info = layout_flash(model)
        print(
            yaml.safe_dump(
                flash_info_to_dict(info),
                sort_keys=False,
                allow_unicode=True,
                default_flow_style=False,
            )
        )

    except Exception as error:
        print(f"\nERROR: Failed to process model {model}")
        print(f"Reason: {str(error)}")
        print("\nStack trace:")
        traceback.print_exc()
        sys.exit(2)


if __name__ == "__main__":
    main()
