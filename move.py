from adafruit_servokit import ServoKit
import time
import sys

import utils

CHANNELS = 16
kit = ServoKit(channels=CHANNELS)

utils.init_servos(kit)

servos = dict()

for i in range(1, len(sys.argv), 2): 
    servo_n = int(sys.argv[i])
    angle = int(sys.argv[i + 1]) 

    if (angle < 0 or angle > 180):
        print(f"Incorrent angle: {angle}")
        sys.exit(1)

    servos[servo_n] = angle

utils.move(kit, servos, 100)
time.sleep(1)
