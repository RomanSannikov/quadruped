import time

import utils


def main():
    kit = utils.init()
    calib_map = utils.read_calibration_data()

    # Walk settings
    x_start = -20
    x_end = 20
    y_ground = -165
    lift = 30
    samples = 5
    steps = 5

    # Move to the initial position
    legs_position = utils.standup(kit, calib_map, x_start, y_ground)
    time.sleep(1)

    for _ in range(steps):
        # forward
        utils.trot(kit, calib_map, x_start, x_end, y_ground, lift, samples)
        # utils.crawl(kit, calib_map, x_start, x_end, y_ground, lift, samples)

        # backwards
        # utils.trot(kit, calib_map, x_end, x_start, y_ground, lift, samples)
        # utils.crawl(kit, calib_map, x_end, x_start, y_ground, lift, samples)

    # Back to default
    time.sleep(1)
    utils.laydown(kit, legs_position)


if __name__ == "__main__":
    main()
