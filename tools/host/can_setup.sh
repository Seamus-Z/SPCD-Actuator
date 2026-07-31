#!/usr/bin/env bash
# Bring SocketCAN FD interface up/down (needs root).
#   sudo bash tools/host/can_setup.sh can0 up
#   sudo bash tools/host/can_setup.sh can0 down
set -euo pipefail

IFACE="${1:-can0}"
ACTION="${2:-up}"
BITRATE="${BITRATE:-1000000}"
DBITRATE="${DBITRATE:-2000000}"

if [[ "$(id -u)" -ne 0 ]]; then
  echo "请用 sudo 运行: sudo bash $0 $IFACE $ACTION" >&2
  exit 1
fi

if [[ ! -d "/sys/class/net/${IFACE}" ]]; then
  echo "${IFACE} 不存在" >&2
  exit 1
fi

case "${ACTION}" in
  up)
    ip link set "${IFACE}" down || true
    ip link set "${IFACE}" type can bitrate "${BITRATE}" dbitrate "${DBITRATE}" fd on
    ip link set "${IFACE}" up
    ip -br link show "${IFACE}"
    ;;
  down)
    ip link set "${IFACE}" down
    ip -br link show "${IFACE}"
    ;;
  *)
    echo "用法: $0 <iface> up|down" >&2
    exit 1
    ;;
esac
