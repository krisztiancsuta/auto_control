#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
BOARD="${1:-pico_w}"
BUILD_DIR="${PROJECT_DIR}/build"

if [[ -z "${PICO_SDK_PATH:-}" ]]; then
  echo "PICO_SDK_PATH is not set."
  echo "Run ./scripts/setup_pico_sdk.sh then source your shell rc file."
  exit 1
fi

has_nosys_specs() {
  local gcc_bin="$1"
  local specs_path
  specs_path="$(${gcc_bin} -print-file-name=nosys.specs 2>/dev/null || true)"
  [[ -n "${specs_path}" && -f "${specs_path}" ]]
}

pick_toolchain_bin_dir() {
  local current_gcc
  current_gcc="$(command -v arm-none-eabi-gcc || true)"

  local candidates=()
  local app_toolchain_glob
  for app_toolchain_glob in /Applications/ArmGNUToolchain/*/arm-none-eabi/bin; do
    if [[ -d "${app_toolchain_glob}" ]]; then
      candidates+=("${app_toolchain_glob}")
    fi
  done
  if [[ -n "${current_gcc}" ]]; then
    candidates+=("$(dirname "${current_gcc}")")
  fi
  candidates+=("/opt/homebrew/bin" "/usr/local/bin")

  local dir
  for dir in "${candidates[@]}"; do
    if [[ -x "${dir}/arm-none-eabi-gcc" ]] && has_nosys_specs "${dir}/arm-none-eabi-gcc"; then
      echo "${dir}"
      return 0
    fi
  done

  return 1
}

TOOLCHAIN_BIN_DIR="$(pick_toolchain_bin_dir || true)"
if [[ -z "${TOOLCHAIN_BIN_DIR}" ]]; then
  echo "No usable arm-none-eabi-gcc toolchain found (missing nosys.specs)."
  echo "Install ARM GNU Toolchain and ensure arm-none-eabi-gcc is available in PATH."
  exit 1
fi

export PATH="${TOOLCHAIN_BIN_DIR}:${PATH}"
echo "Using ARM toolchain from ${TOOLCHAIN_BIN_DIR}"

# Keep one build directory and always recreate it to avoid stale SDK cache.
rm -rf "${BUILD_DIR}"

cmake -S "${PROJECT_DIR}" -B "${BUILD_DIR}" -G Ninja -DPICO_BOARD="${BOARD}"
cmake --build "${BUILD_DIR}" -j

echo
echo "Build finished for board '${BOARD}'. UF2 file: ${BUILD_DIR}/pio_quadrature_encoder.uf2"
