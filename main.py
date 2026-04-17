from adafruit_servokit import ServoKit
import time
import sys
import csv

import utils

def init():
    kit = ServoKit(channels=16)

    utils.init_servos(kit)
    utils.move_servos_to_default_position(kit)

    time.sleep(1)

    return kit

def main():
    kit = init()

    calib_map = []
    with open("servo_to_foot.csv", "r") as file:
        reader = csv.reader(file)
        next(reader)
        for row in reader:
            calib_map.append(tuple(int(item) for item in row))

    x_start = -10
    x_end = 25
    y_ground = -130
    lift = 50

    # Move to the initial position
    hip_angle = utils.find_best_servo_pair(x_start, y_ground, calib_map)["hip"]
    knee_angle = utils.find_best_servo_pair(x_start, y_ground, calib_map)["knee"]
    legs_position = {0: hip_angle, 1: knee_angle, 3: hip_angle, 4: knee_angle, 6: hip_angle, 7: knee_angle, 9: hip_angle, 10: knee_angle}
    utils.move(kit, legs_position, 100)

    legs = [
        {"hip": 0,  "knee": 1},
        {"hip": 3,  "knee": 4},
        {"hip": 6,  "knee": 7},
        {"hip": 9,  "knee": 10},
    ]

    group_A = [0, 3]
    group_B = [1, 2]

    RIGHT_LEGS = {1, 3}

    swing = utils.swing_trajectory(x_start, x_end, y_ground, lift, steps=15)
    stance = utils.stance_trajectory(x_end, x_start, y_ground, steps=15)

    for _ in range(4):
        # --- Phase A ---
        for i in range(len(swing)):
            servo_targets = {}

            for leg_id, leg in enumerate(legs):
                if leg_id in group_A:
                    x, y = swing[i]
                    if leg_id in RIGHT_LEGS:
                        x = -x
                else:
                    x, y = stance[i]

                best = utils.find_best_servo_pair(x, y, calib_map)

                servo_targets[leg["hip"]] = best["hip"]
                servo_targets[leg["knee"]] = best["knee"]

            utils.move(kit, servo_targets, 100)

        # --- Phase B ---
        for i in range(len(swing)):
            servo_targets = {}

            for leg_id, leg in enumerate(legs):
                if leg_id in group_B:
                    x, y = swing[i]
                    if leg_id in RIGHT_LEGS:
                        x = -x
                else:
                    x, y = stance[i]

                best = utils.find_best_servo_pair(x, y, calib_map)

                servo_targets[leg["hip"]] = best["hip"]
                servo_targets[leg["knee"]] = best["knee"]

            utils.move(kit, servo_targets, 100)

    legs_position = {0: hip_angle, 1: knee_angle, 3: hip_angle, 4: knee_angle, 6: hip_angle, 7: knee_angle, 9: hip_angle, 10: knee_angle}
    utils.move(kit, legs_position, 100)
    utils.move_servos_to_default_position(kit)

    #for i in range(3):
    #    trajectory = utils.swing_trajectory(x_start = x_start, x_end = x_end, y_ground = y_ground, lift = lift, steps = 15)
    #    for x, y in trajectory:
    #        best = utils.find_best_servo_pair(x, y, calib_map)
    #        servo_targets = {
    #            HIP: best["hip"],
    #            KNEE: best["knee"]
    #        }
    #        utils.move(kit, servo_targets, 70)

    #    trajectory = utils.stance_trajectory(x_end, x_start, y_ground, steps=5)
    #    for x, y in trajectory:
    #        best = utils.find_best_servo_pair(x, y, calib_map)
    #        servo_targets = {
    #            HIP: best["hip"],
    #            KNEE: best["knee"]
    #        }
    #        utils.move(kit, servo_targets, 80)

# === #

if __name__ == "__main__":
    main()

