#!/usr/bin/env python3
"""Regenerate compile_commands.json for clangd (IDE only — not used by Bazel).

Uses arm-none-eabi-g++ as the compile-database driver so clangd can extract
the embedded libstdc++/newlib include paths via --query-driver. This does not
change the firmware build toolchain.

Headers live at fw/<pkg>/inc/<pkg>/foo.h so #include \"<pkg>/foo.h\"
works with -Ifw/<pkg>/inc (same as Bazel includes=[\"inc\"]).

Usage:
  tools/gen_compile_commands.py
"""

from __future__ import annotations

import json
import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

# Prefer PATH; fall back to the common Debian/Ubuntu location.
COMPILER = shutil.which("arm-none-eabi-g++") or "/usr/bin/arm-none-eabi-g++"

SOURCES = [
    "fw/HAL/src/fdcan.cc",
    "fw/HAL/src/phase_current_adc.cc",
    "fw/HAL/src/phase_pwm.cc",
    "fw/HAL/src/system_clock.cc",
    "fw/app/main.cpp",
    "fw/app/src/application.cc",
    "fw/board/xtellar_stm32g4/firmware_composition.cc",
    "fw/board/xtellar_stm32g4/platform_service.cc",
    "fw/board/xtellar_stm32g4/runtime_config_service.cc",
    "fw/device/src/drv8353s.cc",
    "fw/middleware/src/communication/binary_link.cc",
    "fw/middleware/src/communication/can_command_adapter.cc",
    "fw/middleware/src/communication/telemetry_publisher.cc",
    "fw/middleware/src/control/motor_control_service.cc",
    "fw/bootloader/src/BL_CanDriver.cc",
    "fw/bootloader/src/BL_CommandServer.cc",
    "fw/bootloader/src/BL_FlashWriter.cc",
    "fw/bootloader/src/bootloader.cc",
    "fw/bootloader/src/enter_bootloader.cc",
]

INCLUDES = [
    "-Ifw",
    "-Ifw/HAL/inc",
    "-Ifw/app/inc",
    "-Ifw/bootloader/inc",
    "-Ifw/core/inc",
    "-Ifw/device/inc",
    "-Ifw/math/inc",
    "-Ifw/middleware/inc",
    "-Ifw/ports/inc",
    "-Ifw/protocol/inc",
    "-Ithird_party/mbed-g4",
    "-Ithird_party/mbed-g4/cmsis",
    "-Ithird_party/mbed-g4/cmsis/TARGET_CORTEX_M",
    "-Ithird_party/mbed-g4/hal",
    "-Ithird_party/mbed-g4/platform",
    "-Ithird_party/mbed-g4/drivers",
    "-Ithird_party/mbed-g4/targets/TARGET_STM",
    "-Ithird_party/mbed-g4/targets/TARGET_STM/device",
    "-Ithird_party/mbed-g4/targets/TARGET_STM/TARGET_STM32G4",
    "-Ithird_party/mbed-g4/targets/TARGET_STM/TARGET_STM32G4/device",
    "-Ithird_party/mbed-g4/targets/TARGET_STM/TARGET_STM32G4/TARGET_STM32G474xE",
    "-Ithird_party/mbed-g4/targets/TARGET_STM/TARGET_STM32G4/TARGET_STM32G474xE/device",
    "-Ithird_party/mbed-g4/targets/TARGET_STM/TARGET_STM32G4/TARGET_STM32G474xE/TARGET_NUCLEO_G474RE",
    "-Ithird_party/mbed-g4/targets/TARGET_STM/TARGET_STM32G4/TARGET_STM32G474xE/TARGET_NUCLEO_G474RE/device",
]

DEFINES = [
    "-DSTM32G474xx",
    "-DUSE_HAL_DRIVER",
    "-DTARGET_STM",
    "-DTARGET_STM32G4",
    "-DTARGET_STM32G474xE",
    "-DTARGET_NUCLEO_G474RE",
    "-DSTM32G4",
]


def main() -> None:
    # Do not pass --target=... here: arm-none-eabi-g++ rejects clang's -target,
    # which breaks clangd's --query-driver include extraction.
    db = [
        {
            "directory": str(ROOT),
            "file": rel,
            "arguments": [
                COMPILER,
                "-mcpu=cortex-m4",
                "-mthumb",
                "-c",
                rel,
                "-std=c++17",
                "-ffreestanding",
                "-fno-exceptions",
                "-fno-rtti",
                *INCLUDES,
                *DEFINES,
            ],
        }
        for rel in SOURCES
    ]
    out = ROOT / "compile_commands.json"
    out.write_text(json.dumps(db, indent=2) + "\n")
    print(f"wrote {out.relative_to(ROOT)} ({len(db)} entries)")
    print(f"IDE driver: {COMPILER}")


if __name__ == "__main__":
    main()
