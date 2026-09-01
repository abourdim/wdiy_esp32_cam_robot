#!/usr/bin/env bash
# ota_flash.sh — push firmware to the robot over WiFi OTA
#
# Usage: ./ota_flash.sh <robot-ip> [path/to/firmware.bin]
#
#   No image given   -> builds from current source and uploads (pio run -t upload)
#   Image path given -> uploads that exact .bin directly, no rebuild
#                        (e.g. a previously built .pio/build/esp32cam/firmware.bin)
#
# ArduinoOTA runs whenever the robot has WiFi up, whether that's its own
# "wdiy1" AP (192.168.4.1) or a router it joined -- no special mode needed.
# See README.md's OTA section.

set -e

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$PROJECT_DIR"

if [ -z "${1:-}" ]; then
    echo "Usage: $0 <robot-ip> [path/to/firmware.bin]"
    echo
    echo "Default IP on the robot's own AP (wdiy1) is 192.168.4.1."
    echo "On a router, read the IP off the serial monitor at boot."
    exit 1
fi

CAR_IP="$1"
IMAGE="${2:-}"

if [ -z "$IMAGE" ]; then
    echo "Building from source and flashing $CAR_IP over WiFi OTA..."
    exec pio run -e esp32cam-ota -t upload --upload-port "$CAR_IP"
fi

if [ ! -f "$IMAGE" ]; then
    echo "[ERR] Image not found: $IMAGE" >&2
    exit 1
fi

ESPOTA="$HOME/.platformio/packages/framework-arduinoespressif32/tools/espota.py"
if [ ! -f "$ESPOTA" ]; then
    echo "[ERR] espota.py not found at $ESPOTA — is the espressif32 platform installed?" >&2
    exit 1
fi

echo "Flashing $IMAGE to $CAR_IP over WiFi OTA (no rebuild)..."
python "$ESPOTA" -i "$CAR_IP" -f "$IMAGE" -r
