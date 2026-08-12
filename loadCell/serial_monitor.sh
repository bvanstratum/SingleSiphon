#!/usr/bin/env bash
# Quick serial monitor for the load cell ESP32 — uses the stable by-id path
# (not /dev/ttyACMn) so this always targets the load cell board specifically,
# even if the actuator board is also plugged in.
#
# Logs to loadCell/serial_logs/ so the output can be read back later (e.g.
# handed to Claude) without needing a live screen hardcopy dump.
#
# To exit screen: Ctrl+A then k, then y to confirm (or Ctrl+A then \).
#
#loadcell
PORT="/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_E0:72:A1:FA:98:2C-if00"

#actuator
#PORT="/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_E0:72:A1:FB:D3:D8-if00"
LOG_DIR="$(dirname "$0")/serial_logs"
mkdir -p "$LOG_DIR"
LOG="$LOG_DIR/loadcell_$(date +%Y%m%d_%H%M%S).log"
echo "Logging to: $LOG"
screen -L -Logfile "$LOG" "$PORT" 115200
