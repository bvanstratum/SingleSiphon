#!/usr/bin/env bash
# Quick serial monitor for the motor/actuator ESP32's debug output - but NOT
# connected directly to that board's own USB port. SerialDebug (see
# esp32_micro_ros.ino) goes out over UART1 (D8/D9) to a separate, dedicated
# relay ESP32 wired to those pins, which exposes it as its own USB serial
# device - this script targets THAT relay, not the motor board itself
# (reverted back to this UART1+relay setup after native-USB logging turned
# out to hit an unresolved HWCDC driver bug under load - see
# esp32_micro_ros.ino's SerialDebugRaw comment for the full story).
# Stable by-id path (not /dev/ttyACMn) so this always targets the relay
# specifically, regardless of what else is plugged in or plug order.
#
# Logs to serial_logs/ (alongside this script) so the output can be read
# back later (e.g. handed to Claude) without needing a live screen hardcopy.
#
# To exit screen: Ctrl+A then k, then y to confirm (or Ctrl+A then \).
PORT="/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_D8:3B:DA:46:D3:10-if00"
LOG_DIR="$(dirname "$0")/serial_logs"
mkdir -p "$LOG_DIR"
LOG="$LOG_DIR/motor_$(date +%Y%m%d_%H%M%S).log"
echo "Logging to: $LOG"
screen -L -Logfile "$LOG" "$PORT" 921600
