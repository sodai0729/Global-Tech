import sys
import cv2 as cv
import numpy as np

sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils

rc = racecar_core.create_racecar()


MIN_CONTOUR_AREA = 30
CROP_FLOOR = ((360, 0), (rc.camera.get_height(), rc.camera.get_width()))

BLUE = ((90, 50, 50), (120, 255, 255))

Kp = 1.2
Ki = 0.0
Kd = 0.4


speed = 0.0
angle = 0.0

contour_center = None
contour_area = 0

integral = 0
prev_error = 0


def update_contour():
    global contour_center, contour_area

    image = rc.camera.get_color_image()

    if image is None:
        contour_center = None
        contour_area = 0
        return

    image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])

    contours = rc_utils.find_contours(image, BLUE[0], BLUE[1])
    contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)

    if contour is not None:
        contour_center = rc_utils.get_contour_center(contour)
        contour_area = rc_utils.get_contour_area(contour)
    else:
        contour_center = None
        contour_area = 0

    rc.display.show_color_image(image)

def start():
    global speed, angle, integral, prev_error

    speed = 0
    angle = 0
    integral = 0
    prev_error = 0

    rc.drive.set_speed_angle(speed, angle)
    rc.set_update_slow_time(0.5)

    print(">> PID Line Following START")


def update():
    global speed, angle
    global integral, prev_error

    update_contour()

    if contour_center is not None:
        width = rc.camera.get_width()

        error = rc_utils.remap_range(contour_center[1], 0, width, -1, 1)

        P = Kp * error

        integral += error
        integral = rc_utils.clamp(integral, -1, 1)
        I = Ki * integral

        derivative = error - prev_error
        D = Kd * derivative

        angle = P + I + D

        prev_error = error

    else:
        angle *= 0.9

    angle = rc_utils.clamp(angle, -1, 1)

    rt = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    lt = rc.controller.get_trigger(rc.controller.Trigger.LEFT)

    base_speed = rt - lt

    speed = base_speed * (1 - abs(angle))

    if base_speed > 0:
        speed = max(speed, 0.2)

    rc.drive.set_speed_angle(speed, angle)

    if rc.controller.is_down(rc.controller.Button.A):
        print(f"Speed: {speed:.2f}, Angle: {angle:.2f}")

    if rc.controller.is_down(rc.controller.Button.B):
        if contour_center is None:
            print("No contour")
        else:
            print(f"Center: {contour_center}, Area: {contour_area}")

def update_slow():
    if rc.camera.get_color_image() is None:
        print("X" * 10 + " (No image) " + "X" * 10)
    else:
        if contour_center is None:
            print("-" * 32 + f" : area = {contour_area}")
        else:
            s = ["-"] * 32
            s[int(contour_center[1] / 20)] = "|"
            print("".join(s) + f" : area = {contour_area}")


if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
