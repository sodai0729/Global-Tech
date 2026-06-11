import sys
sys.path.insert(1, "../../library")

import racecar_core
import racecar_utils as rc_utils

rc = racecar_core.create_racecar()

KP = 0.06                 
TARGET_DISTANCE = 60  

FRONT_THRESHOLD = 30      
MAX_SPEED = 1.0


speed = 0.0
angle = 0.0


def start():
    global speed, angle
    speed = 0
    angle = 0

    rc.drive.set_speed_angle(speed, angle)
    print(">> Wall Following START")


def get_valid_distance(scan, angle_index):
    d = scan[angle_index]

    if d == 0 or d > 300:
        return None
    return d


def update():
    global speed, angle

    scan = rc.lidar.get_samples()

    left_vals = []
    for i in [85, 90, 95]:
        d = get_valid_distance(scan, i)
        if d is not None:
            left_vals.append(d)

    if len(left_vals) == 0:
        left = TARGET_DISTANCE
    else:
        left = sum(left_vals) / len(left_vals)

    front = get_valid_distance(scan, 0)
    if front is None:
        front = 100


    error = left - TARGET_DISTANCE
    angle = KP * error


    angle = rc_utils.clamp(angle, -1, 1)


    rt = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    lt = rc.controller.get_trigger(rc.controller.Trigger.LEFT)

    base_speed = rt - lt

    speed = base_speed * (1 - abs(angle))

    if base_speed > 0:
        speed = max(speed, 0.25)

    speed = rc_utils.clamp(speed, -MAX_SPEED, MAX_SPEED)
    rc.drive.set_speed_angle(speed, angle)


    if rc.controller.is_down(rc.controller.Button.A):
        print(f"Left: {left:.1f}  Front: {front:.1f}  Error: {error:.2f}  Angle: {angle:.2f}")

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()
