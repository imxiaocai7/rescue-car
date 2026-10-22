"""Read-only ELF/vector check. This cannot verify physical hardware."""

import hashlib
import argparse
import json
from pathlib import Path
import struct

from elftools.elf.elffile import ELFFile

ROOT = Path(__file__).resolve().parents[1]


def main():
    arguments = argparse.ArgumentParser()
    arguments.add_argument("--environment", choices=("stm32-uart-check", "stm32-motion"), default="stm32-uart-check")
    args = arguments.parse_args()
    motion = args.environment == "stm32-motion"
    directory = ROOT / "firmware/stm32/.pio/build" / args.environment
    packet = (directory / "firmware.bin").read_bytes()
    if len(packet) < 256 or len(packet) > 512 * 1024:
        raise ValueError("unexpected firmware size")
    stack, reset = struct.unpack_from("<II", packet)
    if not 0x20000000 < stack <= 0x20020000 or stack % 8:
        raise ValueError("initial stack outside normal SRAM")
    if not reset & 1 or not 0x08000000 <= (reset & ~1) < 0x08000000 + len(packet):
        raise ValueError("invalid reset vector")
    with (directory / "firmware.elf").open("rb") as stream:
        elf = ELFFile(stream)
        if elf["e_machine"] != "EM_ARM" or not elf.little_endian:
            raise ValueError("not little-endian ARM firmware")
        vectors = elf.get_section_by_name(".isr_vector")
        if vectors is None or vectors["sh_addr"] != 0x08000000:
            raise ValueError("unexpected vector placement")
        symbols = elf.get_section_by_name(".symtab")
        names = {symbol.name: symbol for symbol in symbols.iter_symbols()}
        for handler, vector in (("SysTick_Handler", 15), ("USART2_IRQHandler", 54)):
            symbol = names[handler]
            value = struct.unpack_from("<I", packet, vector * 4)[0]
            if symbol["st_info"]["bind"] == "STB_WEAK" or (symbol["st_value"] & ~1) != (value & ~1):
                raise ValueError(f"handler not installed: {handler}")
        if not motion and any(name.startswith("HAL_TIM_") for name in names):
            raise ValueError("unexpected timer-control implementation")
        if motion and ("cm_control_tick" not in names or "HAL_TIM_PWM_Start" not in names):
            raise ValueError("motion control/PWM missing")
        for segment in elf.iter_segments():
            if segment["p_type"] != "PT_LOAD":
                continue
            start = segment["p_vaddr"]
            end = start + segment["p_memsz"]
            if not ((0x08000000 <= start <= end <= 0x08080000) or
                    (0x20000000 <= start <= end <= 0x20020000)):
                raise ValueError("load segment outside allowed MCU memory")
    if not (directory / "firmware.hex").is_file():
        raise ValueError("HEX was not exported")
    print(json.dumps({"target": "STM32F407VET6", "bin_bytes": len(packet),
                      "sha256": hashlib.sha256(packet).hexdigest(),
                      "initial_stack": hex(stack), "hardware_verified": False,
                      "motor_control_implemented": motion}, indent=2))


if __name__ == "__main__":
    main()
