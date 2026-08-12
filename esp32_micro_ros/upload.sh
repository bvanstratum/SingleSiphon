#!/usr/bin/env bash
set -e

FQBN="esp32:esp32:XIAO_ESP32S3:PSRAM=opi"
PORT="/dev/ttyACM0"
SKETCH_DIR="esp32_micro_ros"

echo "Using sketch dir: $SKETCH_DIR"

arduino-cli compile --fqbn "$FQBN" "$SKETCH_DIR"
arduino-cli upload  --fqbn "$FQBN" -p "$PORT" "$SKETCH_DIR"