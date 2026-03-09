import os

# Explicitly specify board type to ensure correct Blinka/Jetson.GPIO detection
# Board: NVIDIA Jetson Orin Nano Engineering Reference Developer Kit Super
os.environ['BLINKA_JETSON_BOARD'] = 'JETSON_ORIN_NANO'

import board
import busio
import adafruit_pca9685
import curses
import argparse

DEFAULT_CHANNEL = 8
DEFAULT_FREQ = 50  # Hz — standard for ESCs


def us_to_duty_cycle(pulse_us: int, freq: int) -> int:
    """Convert a pulse width in microseconds to a 16-bit PCA9685 duty cycle value."""
    period_us = 1_000_000 / freq
    return int(pulse_us / period_us * 65535)


def safe_addstr(stdscr, row: int, col: int, text: str) -> None:
    max_y, max_x = stdscr.getmaxyx()
    if row >= max_y or col >= max_x:
        return
    # Truncate text so it never runs off the right edge
    max_len = max_x - col - 1
    if max_len <= 0:
        return
    try:
        stdscr.addstr(row, col, text[:max_len])
    except curses.error:
        pass


def draw(stdscr, pulse_us: int, channel: int, freq: int, msg: str = "") -> None:
    stdscr.clear()
    max_y, max_x = stdscr.getmaxyx()

    if max_y < 16 or max_x < 52:
        safe_addstr(stdscr, 0, 0, f"Terminal too small ({max_x}x{max_y}). Need at least 52x16.")
        stdscr.refresh()
        return

    bar_len = 40
    filled = int(pulse_us / 2000 * bar_len)
    bar = "[" + "#" * filled + "-" * (bar_len - filled) + "]"

    safe_addstr(stdscr, 0,  0, "=" * 52)
    safe_addstr(stdscr, 1,  0, "            ESC CALIBRATOR")
    safe_addstr(stdscr, 2,  0, "=" * 52)
    safe_addstr(stdscr, 3,  0, f"  Channel      : {channel}")
    safe_addstr(stdscr, 4,  0, f"  Frequency    : {freq} Hz")
    safe_addstr(stdscr, 5,  0, f"  Pulse Width  : {pulse_us:4d} us  (0 - 2000 us)")
    safe_addstr(stdscr, 6,  0, f"  {bar}  {pulse_us / 2000 * 100:5.1f}%")
    safe_addstr(stdscr, 8,  0, "-" * 52)
    safe_addstr(stdscr, 9,  0, "  UP   / DOWN  : +1 us  / -1 us")
    safe_addstr(stdscr, 10, 0, "  PgUp / PgDn  : +10 us / -10 us")
    safe_addstr(stdscr, 11, 0, "  Home         : set to 0 us")
    safe_addstr(stdscr, 12, 0, "  End          : set to 2000 us")
    safe_addstr(stdscr, 13, 0, "  q            : quit (resets to 1000 us)")
    safe_addstr(stdscr, 14, 0, "-" * 52)
    if msg:
        safe_addstr(stdscr, 15, 0, f"  {msg}")
    stdscr.refresh()


def calibrate(stdscr, channel: int, freq: int) -> None:
    curses.curs_set(0)
    stdscr.keypad(True)

    i2c = busio.I2C(board.SCL, board.SDA)
    pca = adafruit_pca9685.PCA9685(i2c)
    pca.frequency = freq

    pulse_us = 0  # Start at neutral / mid-point

    def set_pulse(us: int) -> None:
        pca.channels[channel].duty_cycle = us_to_duty_cycle(us, freq)

    set_pulse(pulse_us)
    draw(stdscr, pulse_us, channel, freq)

    try:
        while True:
            key = stdscr.getch()

            if key == curses.KEY_UP:
                pulse_us = min(2000, pulse_us + 10)
            elif key == curses.KEY_DOWN:
                pulse_us = max(0, pulse_us - 10)
            elif key == curses.KEY_PPAGE:   # Page Up
                pulse_us = min(2000, pulse_us + 10)
            elif key == curses.KEY_NPAGE:   # Page Down
                pulse_us = max(0, pulse_us - 10)
            elif key == curses.KEY_HOME:
                pulse_us = 0
            elif key == curses.KEY_END:
                pulse_us = 2000
            elif key in (ord('q'), ord('Q')):
                pulse_us = 1000
                set_pulse(pulse_us)
                draw(stdscr, pulse_us, channel, freq, msg="Exiting — pulse reset to 1000 us.")
                stdscr.getch()
                break

            set_pulse(pulse_us)
            draw(stdscr, pulse_us, channel, freq)
    finally:
        pca.deinit()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Interactive ESC calibrator via PCA9685")
    parser.add_argument(
        "--channel", type=int, default=DEFAULT_CHANNEL,
        help=f"PCA9685 channel number (default: {DEFAULT_CHANNEL})"
    )
    parser.add_argument(
        "--freq", type=int, default=DEFAULT_FREQ,
        help=f"PWM frequency in Hz (default: {DEFAULT_FREQ})"
    )
    args = parser.parse_args()

    curses.wrapper(calibrate, args.channel, args.freq)
