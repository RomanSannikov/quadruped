# Quadruped Robot

A 12-DOF quadruped robot controlled by a Raspberry Pi 4B.

## Hardware

- **Raspberry Pi 4B**
- **Adafruit PCA9685** 16-channel PWM servo driver (I2C)
- **12x servos** — 3 per leg (hip, knee, abduction)

## Software

Python control using `adafruit-circuitpython-servokit`.

## Files

| File | Purpose |
|------|---------|
| `main.py` | Run gaits (trot / crawl) |
| `utils.py` | Servo init, calibration, trajectory generation, movement helpers |
| `move.py` | Move individual servos by channel and angle |
| `calibrate.py` | Set servos to default or custom angles for calibration |
| `maxspeed.py` | Measure servo speed |
| `webserver.py` | Simple HTTP server for remote robot control via browser |
| `index.html` | Web interface for controlling the robot with arrow keys |
| `servo_to_foot.csv` | Calibration data mapping hip/knee angles to foot (x, y) positions |

## Setup

```bash
python3 -m venv servokit-env
source servokit-env/bin/activate
pip install -r requirements.txt
```

## Usage

Run a gait:
```bash
python3 main.py
```

Move a servo manually (logical angles):
```bash
python3 move.py 0 90 3 90 # move servo 0 and 3 to 90
```

Calibrate servos (hardware angles):
```bash
python3 calibrate.py        	# reset to defaults
python3 calibrate.py 0 90 1 60  # set servo 0 to 90 and 1 to 60
```

Run the web server for remote control:
```bash
python3 webserver.py
```
Then open `http://<raspberry-pi-ip>:8000` in a browser and press any arrow key to make the robot trot.
