#!/bin/bash
# Flash xtellar_mbed: ISR vectors (0x08000000) + Bootloader (0x0800C000) + App (0x08010000)
# Memory layout:
#   0x08000000 - 0x0800BFFF: ISR vector table (reset → BootloaderEntry @ 0x0800C001)
#   0x0800C000 - 0x0800FFFF: Bootloader (16KB)
#   0x08010000 - 0x0807FFFF: Application (448KB)
set -e

COMBINED=bazel-bin/fw/xtellar.combined.bin
OPENOCD=/home/zzr/openocd/xpack-openocd-0.12.0-7/bin/openocd

echo "=== Building combined bootloader + application image ==="
tools/bazel build //fw:xtellar.combined

echo "=== Flashing combined image ($(stat --format=%s "$COMBINED") bytes) ==="

$OPENOCD \
  -f interface/cmsis-dap.cfg -f target/stm32g4x.cfg \
  -c "init" \
  -c "reset init" \
  -c "program $COMBINED 0x08000000 verify" \
  -c "reset run" \
  -c "exit"

echo "=== Done ==="
