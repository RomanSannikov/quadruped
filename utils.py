import time
import math
import csv

from adafruit_servokit import ServoKit

HIP_DEFAULT_ANGLE = 90
KNEE_DEFAULT_ANGLE = 60
ABDUCTION_DEFAULT_ANGLE = 95

DEFAULT_LOGICAL_CENTER = [HIP_DEFAULT_ANGLE, KNEE_DEFAULT_ANGLE, ABDUCTION_DEFAULT_ANGLE]

SERVO_CAL = {
    # n: (direction, center)
    # hip
    # knee
    # abduction

    # left legs
    0: ( 1, HIP_DEFAULT_ANGLE - 5),
    1: ( 1, KNEE_DEFAULT_ANGLE),
    2: ( 1, ABDUCTION_DEFAULT_ANGLE),

    9: ( 1, HIP_DEFAULT_ANGLE - 3),
    10:( 1, KNEE_DEFAULT_ANGLE - 5),
    11:(-1, ABDUCTION_DEFAULT_ANGLE - 10),

    # right legs (mirrored)
    3: (-1, HIP_DEFAULT_ANGLE + 5),
    4: (-1, KNEE_DEFAULT_ANGLE + 3),
    5: ( 1, ABDUCTION_DEFAULT_ANGLE),

    6: (-1, HIP_DEFAULT_ANGLE + 5),
    7: (-1, KNEE_DEFAULT_ANGLE - 5),
    8: (-1, ABDUCTION_DEFAULT_ANGLE),
}


def swing_trajectory(x_start, x_end, y_ground, lift, samples):
    traj = []
    for i in range(samples):
        t = i / (samples - 1)
        x = x_start + (x_end - x_start) * t 
        y = y_ground + lift * math.sin(math.pi * t)
        traj.append((x, y)) 
    return traj


def stance_trajectory(x_start, x_end, y_ground, samples):
    traj = []
    for i in range(samples):
        t = i / (samples - 1)
        x = x_start + (x_end - x_start) * t 
        y = y_ground
        traj.append((x, y)) 
    return traj


def logical_to_hardware(n, angle):
    direction, center = SERVO_CAL[n]
    logical_center = DEFAULT_LOGICAL_CENTER[n % 3]
    return center + direction * (angle - logical_center)


def move(kit, servo_targets, speed):
    """Move servos to target positions.
    
    servo_targets contains logical angles where:
    - 60 is neutral for knees (1, 4, 7, 10)
    - 90 is neutral for hips (0, 3, 6, 9)
    - +1 means the same physical direction on all legs (up for knees, forward for hips)
    """
    min_delay = 0.002
    max_delay = 0.05

    # Read current hardware angles
    current = {n: kit.servo[n].angle for n in servo_targets}
    
    # Convert logical targets to hardware angles
    # targets = {n: to_hardware(n, servo_targets[n]) for n in servo_targets}
    targets = {
        n: logical_to_hardware(n, servo_targets[n])
        for n in servo_targets
    }
    
    # Calculate how far each servo needs to move
    deltas = {n: targets[n] - current[n] for n in servo_targets}
    
    # Determine number of samples based on largest movement
    samples = int(max(abs(d) for d in deltas.values()))
    if samples == 0:
        return
    
    # Convert speed (0-100) to delay between samples
    speed = max(0, min(100, speed))
    delay = max_delay - (speed / 100) * (max_delay - min_delay)
    
    # Move servos step by step
    for i in range(1, samples + 1):
        for n in servo_targets:
            angle = current[n] + deltas[n] * i / samples
            # Clamp for safety
            angle = max(0, min(180, angle))
            kit.servo[n].angle = angle
        time.sleep(delay)


def raw_move(kit, servo_targets):
    """Directly set servos to target positions without interpolation."""
    for n in servo_targets:
        kit.servo[n].angle = servo_targets[n]
        print(f"Actual default angle: {servo_targets[n]}")
        time.sleep(0.5)


def find_best_servo_pair(x_target, y_target, calib_map):
    """Find servo angles using inverse distance weighting interpolation.
    
    Uses the 4 nearest calibration points and interpolates based on inverse distance.
    This provides smoother transitions than nearest-neighbor lookup.
    """
    # Calculate distances to all calibration points
    distances = []
    for hip, knee, x, y in calib_map:
        dist = math.sqrt((x - x_target)**2 + (y - y_target)**2)
        distances.append((dist, hip, knee, x, y))
    
    # Sort by distance and get the 4 closest points
    distances.sort(key=lambda d: d[0])
    nearest = distances[:4]

    # If we're very close to a calibration point, just use it
    if nearest[0][0] < 0.1:
        return {
            "hip": nearest[0][1],
            "knee": nearest[0][2],
            "x": nearest[0][3],
            "y": nearest[0][4],
            "distance": nearest[0][0]
        }
    
    # Inverse distance weighting (Shepard's method)
    # Use power of 2 for the weight (inverse square law)
    weights = []
    for dist, hip, knee, x, y in nearest:
        if dist < 0.001:  # Avoid division by zero
            weight = 1000000
        else:
            weight = 1.0 / (dist ** 2)
        weights.append(weight)
    
    # Normalize weights
    total_weight = sum(weights)
    weights = [w / total_weight for w in weights]
    
    # Interpolate hip and knee angles
    hip_interp = sum(w * pt[1] for w, pt in zip(weights, nearest))
    knee_interp = sum(w * pt[2] for w, pt in zip(weights, nearest))
    
    return {
        "hip": round(hip_interp),
        "knee": round(knee_interp),
        "x": x_target,
        "y": y_target,
        "distance": nearest[0][0]
    }


def move_servos_to_default_position(kit, at_once = False):
    default_hardware_positions = {}
    default_software_positions = {}

    for i in range(12):
        _, center = SERVO_CAL[i]
        default_hardware_positions[i] = center
        default_software_positions[i] = DEFAULT_LOGICAL_CENTER[i % 3]

    if at_once:
        move(kit, default_software_positions, speed=100)
    else:
        raw_move(kit, default_hardware_positions)


def standup(kit, calib_map, x_start, y_ground):
    hip_angle = find_best_servo_pair(x_start, y_ground, calib_map)["hip"]
    knee_angle = find_best_servo_pair(x_start, y_ground, calib_map)["knee"]
    legs_position = {0: hip_angle, 1: knee_angle, 3: hip_angle, 4: knee_angle, 6: hip_angle, 7: knee_angle, 9: hip_angle, 10: knee_angle}
    move(kit, legs_position, 100)
    return legs_position


def laydown(kit, legs_position):
    move(kit, legs_position, 100)
    move_servos_to_default_position(kit, at_once=True)


def crawl(kit, calib_map, x_start, x_end, y_ground, lift, samples=15):
    hip_angle = find_best_servo_pair(x_start, y_ground, calib_map)["hip"]
    knee_angle = find_best_servo_pair(x_start, y_ground, calib_map)["knee"]
    legs_position = {0: hip_angle, 1: knee_angle, 3: hip_angle, 4: knee_angle, 6: hip_angle, 7: knee_angle, 9: hip_angle, 10: knee_angle}

    legs = [
        {"hip": 0,  "knee": 1},
        {"hip": 3,  "knee": 4},
        {"hip": 6,  "knee": 7},
        {"hip": 9,  "knee": 10},
    ]

    swing = swing_trajectory(x_start, x_end, y_ground, lift, samples=samples)

    for leg_id in [0, 2, 1, 3]:
        for i in range(len(swing)):
            servo_targets = {}
            x, y = swing[i]
            best = find_best_servo_pair(x, y, calib_map)
            servo_targets[legs[leg_id]["hip"]] = best["hip"]
            servo_targets[legs[leg_id]["knee"]] = best["knee"]
            move(kit, servo_targets, 80)
    time.sleep(0.1)
    move(kit, legs_position, 100)


def trot(kit, calib_map, x_start, x_end, y_ground, lift, samples=15):
    legs = [
        {"hip": 0,  "knee": 1},
        {"hip": 3,  "knee": 4},
        {"hip": 6,  "knee": 7},
        {"hip": 9,  "knee": 10},
    ]

    group_A = [0, 2]
    group_B = [1, 3]

    swing = swing_trajectory(x_start, x_end, y_ground, lift, samples=samples)
    stance = stance_trajectory(x_end, x_start, y_ground, samples=samples)

    # --- Phase A ---
    for i in range(len(swing)):
        servo_targets = {}

        for leg_id, leg in enumerate(legs):
            if leg_id in group_A:
                x, y = swing[i]
            else:
                x, y = stance[i]

            best = find_best_servo_pair(x, y, calib_map)

            servo_targets[leg["hip"]] = best["hip"]
            servo_targets[leg["knee"]] = best["knee"]

        move(kit, servo_targets, 100)

    # --- Phase B ---
    for i in range(len(swing)):
        servo_targets = {}

        for leg_id, leg in enumerate(legs):
            if leg_id in group_B:
                x, y = swing[i]
            else:
                x, y = stance[i]

            best = find_best_servo_pair(x, y, calib_map)

            servo_targets[leg["hip"]] = best["hip"]
            servo_targets[leg["knee"]] = best["knee"]

        move(kit, servo_targets, 100)


def init_servos(kit):
    # Configure all servos with consistent pulse width and actuation range
    for i in range(12):
        kit.servo[i].set_pulse_width_range(500, 2500)
        kit.servo[i].actuation_range = 180


def init():
    kit = ServoKit(channels=16)

    init_servos(kit)
    move_servos_to_default_position(kit, at_once=True)

    time.sleep(1)

    return kit


def read_calibration_data():
    calib_map = []
    with open("servo_to_foot.csv", "r") as file:
        reader = csv.reader(file)
        next(reader)
        for row in reader:
            calib_map.append(tuple(int(item) for item in row))
    return calib_map
