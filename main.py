from adafruit_servokit import ServoKit
import time
import csv

import utils

def init():
    kit = ServoKit(channels=16)

    utils.init_servos(kit)
    utils.move_servos_to_default_position(kit, at_once=True)

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

    # trot(kit, calib_map, x_start=-20, x_end=20, y_ground=-165, lift=30, steps=5)
    crawl(kit, calib_map, x_start=-20, x_end=20, y_ground=-165, lift=30, steps=5)


def crawl(kit, calib_map, x_start, x_end, y_ground, lift, steps=15):
    # Move to the initial position
    hip_angle = utils.find_best_servo_pair(x_start, y_ground, calib_map)["hip"]
    knee_angle = utils.find_best_servo_pair(x_start, y_ground, calib_map)["knee"]
    legs_position = {0: hip_angle, 1: knee_angle, 3: hip_angle, 4: knee_angle, 6: hip_angle, 7: knee_angle, 9: hip_angle, 10: knee_angle}
    utils.move(kit, legs_position, 100)
    time.sleep(1)

    legs = [
        {"hip": 0,  "knee": 1},
        {"hip": 3,  "knee": 4},
        {"hip": 6,  "knee": 7},
        {"hip": 9,  "knee": 10},
    ]

    swing = utils.swing_trajectory(x_start, x_end, y_ground, lift, steps=steps)

    for _ in range(7):
        for leg_id in [0, 2, 1, 3]:
            for i in range(len(swing)):
                servo_targets = {}
                x, y = swing[i]
                best = utils.find_best_servo_pair(x, y, calib_map)
                servo_targets[legs[leg_id]["hip"]] = best["hip"]
                servo_targets[legs[leg_id]["knee"]] = best["knee"]
                utils.move(kit, servo_targets, 80)
        time.sleep(0.1)
        utils.move(kit, legs_position, 100)

    # Back to default
    time.sleep(1)
    legs_position = {0: hip_angle, 1: knee_angle, 3: hip_angle, 4: knee_angle, 6: hip_angle, 7: knee_angle, 9: hip_angle, 10: knee_angle}
    utils.move(kit, legs_position, 100)
    utils.move_servos_to_default_position(kit, at_once=True)

def trot(kit, calib_map, x_start, x_end, y_ground, lift, steps=15):
    # Move to the initial position
    hip_angle = utils.find_best_servo_pair(x_start, y_ground, calib_map)["hip"]
    knee_angle = utils.find_best_servo_pair(x_start, y_ground, calib_map)["knee"]
    legs_position = {0: hip_angle, 1: knee_angle, 3: hip_angle, 4: knee_angle, 6: hip_angle, 7: knee_angle, 9: hip_angle, 10: knee_angle}
    utils.move(kit, legs_position, 100)
    time.sleep(1)

    legs = [
        {"hip": 0,  "knee": 1},
        {"hip": 3,  "knee": 4},
        {"hip": 6,  "knee": 7},
        {"hip": 9,  "knee": 10},
    ]

    group_A = [0, 2]
    group_B = [1, 3]

    swing = utils.swing_trajectory(x_start, x_end, y_ground, lift, steps=steps)
    stance = utils.stance_trajectory(x_end, x_start, y_ground, steps=steps)

    for _ in range(7):
        # --- Phase A ---
        for i in range(len(swing)):
            servo_targets = {}

            for leg_id, leg in enumerate(legs):
                if leg_id in group_A:
                    x, y = swing[i]
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
                else:
                    x, y = stance[i]

                best = utils.find_best_servo_pair(x, y, calib_map)

                servo_targets[leg["hip"]] = best["hip"]
                servo_targets[leg["knee"]] = best["knee"]

            utils.move(kit, servo_targets, 100)

    # Back to default
    time.sleep(1)
    legs_position = {0: hip_angle, 1: knee_angle, 3: hip_angle, 4: knee_angle, 6: hip_angle, 7: knee_angle, 9: hip_angle, 10: knee_angle}
    utils.move(kit, legs_position, 100)
    utils.move_servos_to_default_position(kit, at_once=True)


if __name__ == "__main__":
    main()
