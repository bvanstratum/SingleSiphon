#!/usr/bin/env bash
set -e

FQBN="esp32:esp32:XIAO_ESP32S3:PSRAM=opi"
# Stable by-id path (keyed to this specific chip's USB serial number) rather
# than /dev/ttyACMn — with two identical XIAO ESP32-S3 boards in this
# project, a bare port number could silently point at the wrong board
# depending on plug-in order. Same fix as the camera path issue elsewhere
# in this project.
PORT="/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_E0:72:A1:FA:98:2C-if00"
SKETCH_DIR="esp32LoadCell_mROS"

echo "Using sketch dir: $SKETCH_DIR"

arduino-cli compile --fqbn "$FQBN" "$SKETCH_DIR"
arduino-cli upload  --fqbn "$FQBN" -p "$PORT" "$SKETCH_DIR"
