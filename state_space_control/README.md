# Raspberry Pi Pico PIO Quadrature Encoder

This folder contains a standalone Pico SDK project for Raspberry Pi Pico (RP2040).
It reads a quadrature encoder on two GPIO pins using a PIO state machine, then decodes
position in C.

## Wiring

- Encoder channel A -> GP2
- Encoder channel B -> GP3
- Encoder GND -> Pico GND
- Encoder VCC -> Pico 3V3 (or per encoder spec)

The code enables pull-ups for GP2/GP3.

## 1) macOS environment setup + SDK download

From this folder:

```bash
cd state_space_control
chmod +x scripts/*.sh
./scripts/setup_pico_sdk.sh
source ~/.zshrc
```

What this does:

- Clones `pico-sdk` into `third_party/pico-sdk`
- Installs `cmake`, `ninja`, `arm-none-eabi-gcc`, and `picotool` via Homebrew
- Appends `PICO_SDK_PATH` export to your shell rc file

If Homebrew shows link conflicts for `gcc`, you can ignore that for this project as long as
`arm-none-eabi-gcc` is found in your PATH after `source ~/.zshrc`.

## 2) Build

```bash
./scripts/build.sh pico
```

The build script uses a single `build/` directory and rewrites it on each run.
It also auto-selects a working `arm-none-eabi-gcc` toolchain (with `nosys.specs`).

For Pico 2 (RP2350), use:

```bash
./scripts/build.sh pico2
```

Artifacts are generated in `build/`, including:

- `pio_quadrature_encoder.uf2`
- `pio_quadrature_encoder.elf`

Board-specific output folders are used:

- `build-pico/` for RP2040
- `build-pico2/` for RP2350

## 3) Flash

1. Hold BOOTSEL and plug in Pico 2.
2. Run:

```bash
./scripts/flash.sh pico
```

For Pico 2 builds, flash with:

```bash
./scripts/flash.sh pico2
```

Or copy the UF2 manually to the mounted `RPI-RP2` drive.

## 4) Serial monitor

After reset, the firmware prints encoder counts over USB CDC.

```bash
ls /dev/tty.usbmodem*
screen /dev/tty.usbmodemXXXX 115200
```

## Notes

- Sampling frequency is controlled by `SAMPLE_HZ` in `src/main.c`.
- This implementation uses x1 decode transitions (`-1`, `0`, `+1` per valid state edge).
- Default board is set to `pico` (RP2040) in CMake and build script.
- For RP2350 boards, build with `./scripts/build.sh pico2`.

## Troubleshooting (Homebrew gcc link warnings)

If you see messages like:

`Target /opt/homebrew/lib/gcc/14/gcc is a symlink belonging to gcc`

Use this check first:

```bash
command -v arm-none-eabi-gcc
arm-none-eabi-gcc --version
```

If those commands work, continue with `./scripts/build.sh`.

Only if they do not work, run:

```bash
source ~/.zshrc
brew link arm-none-eabi-gcc
```
