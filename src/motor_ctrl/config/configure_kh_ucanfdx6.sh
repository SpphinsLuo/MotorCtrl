#!/usr/bin/env bash
set -euo pipefail

# KH-UCANFDX6-Mini exposes SocketCAN interfaces through the detected Linux
# CAN driver (this host reports peak_usb and can0..can5).
# This configures the two motor channels used by motor_ctrl. Run as root.
# The motor protocol is classic CAN at 1 Mbit/s, not CAN FD.

BITRATE="${1:-1000000}"
for IFACE in can0 can1; do
  ip link set "$IFACE" down 2>/dev/null || true
  ip link set "$IFACE" type can bitrate "$BITRATE" fd off
  ip link set "$IFACE" up
done

ip -details link show can0
ip -details link show can1
