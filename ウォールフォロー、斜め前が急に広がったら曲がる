import sys
sys.path.insert(1, '../../library')

import racecar_core
rc = racecar_core.create_racecar()

import racecar_utils as rc_utils
import math
import cv2 as cv
import numpy as np


##################################################
# STATE
##################################################

mode = "STRAIGHT"

prev_right_front = 0
prev_left_front = 0

prev_error = 0
integral = 0


##################################################
# PID RESET
##################################################

def reset_pid():
    global prev_error, integral
    prev_error = 0
    integral = 0


##################################################
# STRAIGHT (PID + speed control)
##################################################

def straight(scan):

    right_angle = list(range(20, 51, 5))
    left_angle  = list(range(310, 341, 5))

    right_distance = []
    left_distance = []

    for i in range(len(right_angle)):
        right_distance.append(
            rc_utils.get_lidar_average_distance(scan, right_angle[i], 8)
        )
        left_distance.append(
            rc_utils.get_lidar_average_distance(scan, left_angle[i], 8)
        )

    right_avg = sum(right_distance) / len(right_distance)
    left_avg  = sum(left_distance) / len(left_distance)

    error = right_avg - left_avg

    Kp = 0.007
    Kd = 0.012

    global prev_error

    derivative = error - prev_error
    prev_error = error

    angle = Kp * error + Kd * derivative
    angle = rc_utils.clamp(angle, -1, 1)

    speed = 1.0

    # カーブ気味なら減速
    if abs(error) > 50:
        speed = 0.8

    return angle, speed


##################################################
# TURN
##################################################

def turn_right():
    return 1, 0.6


def turn_left():
    return -1, 0.6


##################################################
# START
##################################################

def start():
    rc.drive.stop()
    print("start")


##################################################
# UPDATE
##################################################

def update():

    global mode
    global prev_right_front, prev_left_front

    scan = rc.lidar.get_samples()

    ##################################################
    # FRONT FEATURES
    ##################################################

    right_front = max(
        rc_utils.get_lidar_average_distance(scan, 55, 5),
        rc_utils.get_lidar_average_distance(scan, 60, 5),
        rc_utils.get_lidar_average_distance(scan, 65, 5)
    )

    left_front = max(
        rc_utils.get_lidar_average_distance(scan, 295, 5),
        rc_utils.get_lidar_average_distance(scan, 300, 5),
        rc_utils.get_lidar_average_distance(scan, 305, 5)
    )

    ##################################################
    # MODE: STRAIGHT
    ##################################################

    if mode == "STRAIGHT":

        angle, speed = straight(scan)

        # カーブ検出（安全版：250 + 2倍条件）
        if right_front > 250 and right_front > prev_right_front * 2:
            mode = "RIGHT"

        elif left_front > 250 and left_front > prev_left_front * 2:
            mode = "LEFT"


    ##################################################
    # MODE: RIGHT TURN
    ##################################################

    elif mode == "RIGHT":

        angle, speed = turn_right()

        right90 = rc_utils.get_lidar_average_distance(scan, 90, 5)
        right45 = rc_utils.get_lidar_average_distance(scan, 45, 5)

        if right90 > 0:
            ratio = right45 / right90

            if ratio < 1.6:
                mode = "STRAIGHT"
                reset_pid()


    ##################################################
    # MODE: LEFT TURN
    ##################################################

    elif mode == "LEFT":

        angle, speed = turn_left()

        left270 = rc_utils.get_lidar_average_distance(scan, 270, 5)
        left315 = rc_utils.get_lidar_average_distance(scan, 315, 5)

        if left270 > 0:
            ratio = left315 / left270

            if ratio < 1.6:
                mode = "STRAIGHT"
                reset_pid()


    ##################################################
    # SAVE STATE
    ##################################################

    prev_right_front = right_front
    prev_left_front = left_front

    rc.drive.set_speed_angle(speed, angle)
