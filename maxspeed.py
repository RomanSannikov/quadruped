from adafruit_servokit import ServoKit
import time

kit = ServoKit(channels=16)

kit.servo[0].set_pulse_width_range(500, 2500)
kit.servo[0].actuation_range = 180

kit.servo[0].angle = 0 
time.sleep(1)

start_angle = 0
end_angle = 90

t0 = time.perf_counter()
kit.servo[0].angle = end_angle
time.sleep(0.5)
t1 = time.perf_counter()

dt = t1 - t0
angle_delta = abs(end_angle - start_angle)

print(f"Moved {angle_delta} in {dt:.3f} s")
print(f"= {dt / (angle_delta / 60):.3f} s per 60")

