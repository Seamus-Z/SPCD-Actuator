#!/bin/bash
# Build the application and update it over CAN-FD.
# Usage: ./can_update.sh [can-interface]
set -euo pipefail

CAN_IF="${1:-can0}"
APP_BIN="bazel-bin/fw/app.bin"

if pgrep -af 'tools/host/gui/web_server.py' >/dev/null; then
  echo "ERROR: tools/host/gui/web_server.py is still running." >&2
  pgrep -af 'tools/host/gui/web_server.py' >&2 || true
  echo "Stop the host server before bootloading so it cannot compete for gs_usb." >&2
  exit 1
fi

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
echo "(Tips: keep web_server.py stopped; use a stiff DC PSU — supply sag" \
     "during Flash page erase can reset the target mid-write.)"
sudo ip link set "${CAN_IF}" txqueuelen 1000 2>/dev/null || true
tools/bootload_test.py \
  --interface "${CAN_IF}" \
  --flash "${APP_BIN}" \
  --verbose

echo "=== CAN update completed and verified ==="
