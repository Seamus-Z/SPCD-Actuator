#!/bin/bash
# Build the application and update it over CAN-FD.
# Usage: ./can_update.sh [can-interface]
set -euo pipefail

CAN_IF="${1:-can0}"
APP_BIN="bazel-bin/fw/app.bin"

echo "=== Building application ==="
tools/bazel build //fw:app_binary

echo "=== Configuring ${CAN_IF}: nominal 1Mbps, data 2Mbps ==="
sudo ip link set "${CAN_IF}" down 2>/dev/null || true
sudo ip link set "${CAN_IF}" type can \
  bitrate 1000000 \
  dbitrate 2000000 \
  fd on
sudo ip link set "${CAN_IF}" up

echo "=== Updating ${APP_BIN} over ${CAN_IF} ==="
tools/bootload_test.py \
  --interface "${CAN_IF}" \
  --flash "${APP_BIN}" \
  --verbose

echo "=== CAN update completed and verified ==="
