 # auto_control

Remote keyboard-controlled RC car using a NVIDIA Jetson Orin Nano and an Adafruit PCA9685 PWM driver.

A server runs on the **control PC** and captures keyboard input. A client runs on the **Jetson** and drives the servo (steering) and ESC (throttle) over a TCP socket connection.

---

## Architecture

```
Control PC  ──(TCP)──►  Jetson Orin Nano
auto_controll_server.py    controll.py
  keyboard input             PCA9685 PWM
  W/A/S/D keys               servo ch4 (steering)
                             ESC ch8 (throttle)
```

---

## ESC Calibration

| Value | Pulse Width | Meaning       |
|-------|-------------|---------------|
| -1.0  | 1500 µs     | Max backward  |
|  0.0  | 1600 µs     | Idle / neutral |
| +1.0  | 1650 µs     | Max forward   |

Use `calibrator.py` to determine these values interactively before the first run.

---

## Setup

### Requirements

- Python 3
- NVIDIA Jetson Orin Nano (for `controll.py` and `calibrator.py`)
- Adafruit PCA9685 PWM driver connected via I2C

### Install

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

---

## Usage

### 1. Calibrate the ESC (Jetson, first time only)

```bash
python3 calibrator.py --channel 8 --freq 50
```

Use arrow keys to adjust pulse width and note the values for backward max, idle, and forward max.

### 2. Start the control server (Control PC)

```bash
python3 auto_controll_server.py --host_ip 0.0.0.0 --host_port 5000
```

### 3. Start the car client (Jetson)

```bash
python3 controll.py --host_ip <CONTROL_PC_IP> --host_port 5000
```

---

## Controls

| Key | Action          |
|-----|-----------------|
| W   | Forward         |
| S   | Backward        |
| A   | Steer left      |
| D   | Steer right     |
| P   | Enable          |
| ESC | Stop / quit     |

---

## Files

| File                      | Description                                      |
|---------------------------|--------------------------------------------------|
| `auto_controll_server.py` | Keyboard listener and TCP server (Control PC)    |
| `controll.py`             | Car controller and TCP client (Jetson)           |
| `calibrator.py`           | Interactive ESC pulse-width calibration tool     |
| `requirements.txt`        | Python dependencies                              |
