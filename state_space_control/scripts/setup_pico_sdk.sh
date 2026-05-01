#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
THIRD_PARTY_DIR="${PROJECT_DIR}/third_party"
SDK_DIR="${THIRD_PARTY_DIR}/pico-sdk"

mkdir -p "${THIRD_PARTY_DIR}"

if [[ ! -d "${SDK_DIR}" ]]; then
  echo "Cloning pico-sdk into ${SDK_DIR}"
  git clone --depth=1 https://github.com/raspberrypi/pico-sdk.git "${SDK_DIR}"
else
  echo "pico-sdk already exists at ${SDK_DIR}"
fi

echo "Initializing submodules"
git -C "${SDK_DIR}" submodule update --init

if ! command -v brew >/dev/null 2>&1; then
  echo "Homebrew is required for dependency installation on macOS."
  echo "Install Homebrew from https://brew.sh and rerun this script."
  exit 1
fi

echo "Installing toolchain dependencies via Homebrew"
brew install cmake ninja arm-none-eabi-gcc picotool >/dev/null

SHELL_NAME="${SHELL##*/}"
if [[ "${SHELL_NAME}" == "zsh" ]]; then
  RC_FILE="${HOME}/.zshrc"
else
  RC_FILE="${HOME}/.bashrc"
fi

EXPORT_LINE="export PICO_SDK_PATH=\"${SDK_DIR}\""
if ! grep -Fq "${EXPORT_LINE}" "${RC_FILE}" 2>/dev/null; then
  echo "${EXPORT_LINE}" >> "${RC_FILE}"
  echo "Added PICO_SDK_PATH export to ${RC_FILE}"
fi

echo
echo "Setup complete."
echo "Run: source \"${RC_FILE}\""
echo "Then build with: ./scripts/build.sh"
