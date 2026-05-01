#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
BOARD="${1:-pico}"
BUILD_DIR="${PROJECT_DIR}/build-${BOARD}"
UF2_FILE="${BUILD_DIR}/pio_quadrature_encoder.uf2"

if [[ ! -f "${UF2_FILE}" ]]; then
  echo "UF2 not found at ${UF2_FILE}. Build first with ./scripts/build.sh ${BOARD}"
  exit 1
fi

# Works when the board is already in BOOTSEL mode and mounted as RPI-RP2.
MOUNT_DIR=$(ls /Volumes | grep -E '^RPI-RP2$' || true)
if [[ -z "${MOUNT_DIR}" ]]; then
  echo "RPI-RP2 volume not found. Hold BOOTSEL while plugging in the Pico 2."
  exit 1
fi

cp "${UF2_FILE}" "/Volumes/${MOUNT_DIR}/"
echo "Flashed ${UF2_FILE} to /Volumes/${MOUNT_DIR}/"
