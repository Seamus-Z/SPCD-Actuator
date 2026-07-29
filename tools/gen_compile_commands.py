#!/usr/bin/env python3
"""Regenerate compile_commands.json for clangd.

Headers live at fw/<pkg>/inc/<pkg>/foo.h so #include \"<pkg>/foo.h\"
works with -Ifw/<pkg>/inc (same as Bazel includes=[\"inc\"]).

Usage:
  tools/gen_compile_commands.py
"""

from __future__ import annotations

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

SOURCES = [
    "fw/HAL/src/fdcan.cc",
    "fw/HAL/src/phase_current_adc.cc",
    "fw/HAL/src/phase_pwm.cc",
    "fw/app/main.cpp",
    "fw/app/src/application.cc",
    "fw/app/src/app_telemetry.cc",
    "fw/device/src/drv8353s.cc",
    "fw/telemetry/src/diagnostic_server.cc",
    "fw/bootloader/src/BL_CanDriver.cc",
    "fw/bootloader/src/BL_CommandServer.cc",
    "fw/bootloader/src/BL_FlashWriter.cc",
    "fw/bootloader/src/bootloader.cc",
    "fw/bootloader/src/enter_bootloader.cc",
]

INCLUDES = [
    "-Ifw",
    "-Ifw/HAL/inc",
    "-Ifw/device/inc",
    "-Ifw/control/inc",
    "-Ifw/math/inc",
    "-Ifw/telemetry/inc",
    "-Ifw/pool/inc",
    "-Ifw/app",
    "-Ifw/app/inc",
    "-Ifw/bootloader/inc",
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
    db = [
        {
            "directory": str(ROOT),
            "file": rel,
            "arguments": [
                "clang++",
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


if __name__ == "__main__":
    main()
