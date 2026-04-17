from adafruit_servokit import ServoKit
import time
import math

SERVO_CAL = {
    # n: (direction, center)

    # left legs
    0: ( 1, 90),   # hip example
    1: ( 1, 60),   # knee
    2: ( 1, 95),

    6: ( 1, 90),
    7: ( 1, 60),
    8: ( 1, 95),

    # right legs (mirrored)
    3: (-1, 90),
    4: (-1, 60),
    5: ( 1, 95),

    9: (-1, 90),
    10:(-1, 60),
    11:( 1, 95),
}


def swing_trajectory(x_start, x_end, y_ground, lift, steps):
    traj = []
    for i in range(steps):
        t = i / (steps - 1)
        x = x_start + (x_end - x_start) * t 
        y = y_ground + lift * math.sin(math.pi * t)
        traj.append((x, y)) 
    return traj


def stance_trajectory(x_start, x_end, y_ground, steps):
    traj = []
    for i in range(steps):
        t = i / (steps - 1)
        x = x_start + (x_end - x_start) * t 
        y = y_ground
        traj.append((x, y)) 
    return traj


def logical_to_hardware(n, angle):
    direction, center = SERVO_CAL[n]
    return center + direction * (angle - center)


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
    
    # Determine number of steps based on largest movement
    steps = int(max(abs(d) for d in deltas.values()))
    if steps == 0:
        return
    
    # Convert speed (0-100) to delay between steps
    speed = max(0, min(100, speed))
    delay = max_delay - (speed / 100) * (max_delay - min_delay)
    
    # Move servos step by step
    for i in range(1, steps + 1):
        for n in servo_targets:
            angle = current[n] + deltas[n] * i / steps
            # Clamp for safety
            angle = max(0, min(180, angle))
            kit.servo[n].angle = angle
        time.sleep(delay)


def find_best_servo_pair(x_target, y_target, calib_map):
    best = None
    best_dist2 = float("inf")

    for hip, knee, x, y in calib_map:
        dist2 = (x - x_target)**2 + (y - y_target)**2
        if dist2 < best_dist2:
            best_dist2 = dist2
            best = {
                "hip": hip,
                "knee": knee,
                "x": x,
                "y": y,
                "distance": math.sqrt(dist2)
            }

    return best


def move_servos_to_default_position(kit, speed=70):
    default_positions = {}
    for i in range(0, 12, 3):
        default_positions[i] = 90      # hip
        default_positions[i + 1] = 60  # knee
        default_positions[i + 2] = 95  # abduction
    
    move(kit, default_positions, speed)


def init_servos(kit):
    # Configure all servos with consistent pulse width and actuation range
    for i in range(12):
        kit.servo[i].set_pulse_width_range(500, 2500)
        kit.servo[i].actuation_range = 180
