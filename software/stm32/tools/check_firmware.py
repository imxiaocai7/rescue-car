"""Read-only ELF/BIN/HEX structural checks, not proof of MCU/GPIO operation."""

import argparse
import hashlib
import json
from pathlib import Path
import struct

from elftools.elf.elffile import ELFFile


def verify_firmware(path: Path) -> dict:
    flash_start, flash_end = 0x08000000, 0x08080000
    ram_start, ram_end = 0x20000000, 0x20020000
    with path.open("rb") as handle:
        elf = ELFFile(handle)
        if elf.header["e_machine"] != "EM_ARM":
            raise ValueError("firmware is not ARM ELF")
        vector = elf.get_section_by_name(".isr_vector")
        if vector is None or vector["sh_addr"] != flash_start:
            raise ValueError("vector table must start at 0x08000000")
        data = vector.data()
        stack, reset = struct.unpack_from("<II", data)
        if not ram_start < stack <= ram_end or stack % 8:
            raise ValueError("initial stack is outside the 128KiB normal SRAM")
        if not reset & 1 or not flash_start <= (reset & ~1) < flash_end:
            raise ValueError("reset vector must be Thumb code in flash")
        if elf.header["e_entry"] != reset:
            raise ValueError("ELF entry and reset vector differ")
        symbols = elf.get_section_by_name(".symtab")
        if symbols is None:
            raise ValueError("unstripped symbol table required for handler checks")
        for index, name in (
            (1, "Reset_Handler"), (2, "NMI_Handler"), (3, "HardFault_Handler"),
            (4, "MemManage_Handler"), (5, "BusFault_Handler"), (6, "UsageFault_Handler"),
            (15, "SysTick_Handler"), (54, "USART2_IRQHandler"),
        ):
            matches = symbols.get_symbol_by_name(name)
            # ST's CMSIS startup intentionally marks Reset_Handler weak; it
            # still supplies the real data/BSS/SystemInit/main startup path.
            bindings = ("STB_GLOBAL", "STB_WEAK") if index == 1 else ("STB_GLOBAL",)
            if not matches or matches[0]["st_info"]["bind"] not in bindings:
                raise ValueError(f"missing strong handler: {name}")
            target = struct.unpack_from("<I", data, 4 * index)[0]
            if target != matches[0]["st_value"]:
                raise ValueError(f"vector entry does not point at {name}")
        for segment in elf.iter_segments():
            if segment["p_type"] != "PT_LOAD" or segment["p_memsz"] == 0:
                continue
            start, end = segment["p_vaddr"], segment["p_vaddr"] + segment["p_memsz"]
            if not (flash_start <= start < end <= flash_end or ram_start <= start < end <= ram_end):
                raise ValueError("load segment lies outside project flash/SRAM limits")
        for name in ("HAL_TIM_PWM_Start", "HAL_TIM_Encoder_Start", "rover_application_step", "motion_control_update"):
            if symbols.get_symbol_by_name(name):
                raise ValueError(f"motor-control symbol unexpectedly linked: {name}")
    binary = path.with_suffix(".bin").read_bytes()
    hex_file = path.with_suffix(".hex")
    if not 0 < len(binary) <= flash_end - flash_start or not hex_file.is_file():
        raise ValueError("missing/oversized BIN or missing HEX")
    if struct.unpack_from("<II", binary) != (stack, reset):
        raise ValueError("BIN and ELF vector tables differ")
    return {
        "target": "f407-communication-only", "arm_elf": True,
        "flash_base": hex(flash_start), "initial_stack": hex(stack),
        "binary_bytes": len(binary), "sha256_bin": hashlib.sha256(binary).hexdigest(),
        "strong_interrupt_handlers": True, "hardware_verified": False,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("elf", nargs="?", type=Path,
                        default=Path(__file__).resolve().parents[1] / ".pio/build/f407-communication-only/firmware.elf")
    args = parser.parse_args()
    print(json.dumps(verify_firmware(args.elf), indent=2))


if __name__ == "__main__":
    main()
