from adafruit_servokit import ServoKit
import time
import sys

import utils

CHANNELS = 16
kit = ServoKit(channels=CHANNELS)

utils.init_servos(kit)

servos = []

for i in range(1, len(sys.argv), 2):
    servo_n = int(sys.argv[i])
    angle = int(sys.argv[i + 1])

    if (angle < 0 or angle > 180):
        print(f"Incorrent angle: {angle}")
        sys.exit(1)

    servos.append({servo_n: angle})

if not len(servos): # Defaults
    utils.move_servos_to_default_position(kit)
else: # User defined
    print("Actual angles:")
    for item in servos:
        n, angle = item.popitem()
        kit.servo[n].angle = angle
        print(f"{n}: {round(kit.servo[n].angle)}")
        # time.sleep(0.05)
