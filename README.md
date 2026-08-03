# P.A.D.D.S

![./banner.webp](./banner.webp)

P.A.D.D.S is a mechatronic stomatologic system for replicating and observing jaw movement. The project combines Arduino-controlled servos with a Python desktop controller for direct motor positioning, motion presets, biomechanics sequences, and ArUco-based trajectory tracking.

## What It Does

- Controls 8 servo motors from an Arduino Uno.
- Provides a Python UI for live motor sliders and preset jaw positions.
- Runs automated movement sequences such as mastication and Posselt's envelope.
- Tracks relative jaw motion with OpenCV ArUco markers and displays the trajectory.
- Supports recording and scrubbing through captured movement sessions.

## Tech Stack

- Arduino / PlatformIO firmware in `src/main.cpp`
- Python controller in `host/controller.py`
- Serial communication at `115200` baud
- OpenCV ArUco tracking
- `ttkbootstrap` GUI

## Setup

Install the Python dependencies:

```bash
pip install -r host/requirements.txt
```

Install PlatformIO if needed:

```bash
pip install platformio
```

Check the hardware settings in `host/controller.py` before running:

```python
SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200
```

The camera index is currently set in `VideoCaptureThread`:

```python
cap = cv2.VideoCapture(1)
```

## Usage

Upload the Arduino firmware:

```bash
make upload
```

Start the Python controller:

```bash
make python
```

Or upload and launch in one step:

```bash
make run
```

## Control Protocol

The Python UI sends simple serial commands to the Arduino:

```text
motor_index:percentage
```

Example:

```text
3:75
```

The firmware maps each `0-100%` value to that motor's configured open and closed servo angles.

## Motor Mapping

| Index | Motor |
| --- | --- |
| 0 | TEMP STG |
| 1 | TEMP DRP |
| 2 | MAS STG |
| 3 | MAS DRP |
| 4 | LAT STG |
| 5 | LAT DRP |
| 6 | MED STG |
| 7 | MED DRP |

## Notes

- The Arduino starts all motors in their configured closed positions.
- If the serial connection fails, the Python UI still opens in test mode.
- ArUco tracking expects markers `0` and `1` to be visible in the camera feed.

## License

This project is licensed under the MIT License. See `LICENSE` for details.
