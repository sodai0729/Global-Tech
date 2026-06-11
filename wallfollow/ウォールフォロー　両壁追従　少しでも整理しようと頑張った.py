def bothwallfollow1(scan):
    global integral, prev_error, prev_prev_error

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
    Ki = 0.0
    Kd = 0.012

    integral += error
    integral = rc_utils.clamp(integral, -10, 10)

    derivative  = error - prev_error
    derivative2 = prev_error - prev_prev_error

    angle = Kp * error + Kd * (
        derivative * 0.3 +
        derivative2 * 0.7
    )

    angle = rc_utils.clamp(angle, -1, 1)

    prev_prev_error = prev_error
    prev_error = error

    if angle > 0.9 or angle < -0.9:
        speed = 0.8
    else:
        speed = 1

    return angle, speed


def bothwallfollow2(scan):
    global integral, prev_error, prev_prev_error

    right_angle = list(range(40, 61, 5))
    left_angle  = list(range(300, 321, 5))

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

    Kp = 0.0175
    Ki = 0.0
    Kd = 0.03

    integral += error
    integral = rc_utils.clamp(integral, -10, 10)

    derivative  = error - prev_error
    derivative2 = prev_error - prev_prev_error

    angle = Kp * error + Kd * (
        derivative * 0.3 +
        derivative2 * 0.7
    )

    angle = rc_utils.clamp(angle, -1, 1)

    prev_prev_error = prev_error
    prev_error = error

    if angle > 0.9 or angle < -0.9:
        speed = 0.8
    else:
        speed = 1

    return angle, speed


def update():
    global speed, angle

    scan = rc.lidar.get_samples()

    right_front_angle = list(range(40, 51, 5))
    left_front_angle  = list(range(310, 321, 5))

    right_back_angle = list(range(130, 141, 5))
    left_back_angle  = list(range(220, 231, 5))

    right_f = []
    left_f  = []
    right_b = []
    left_b  = []

    for i in range(len(right_front_angle)):
        right_f.append(
            rc_utils.get_lidar_average_distance(
                scan,
                right_front_angle[i],
                8
            )
        )

        left_f.append(
            rc_utils.get_lidar_average_distance(
                scan,
                left_front_angle[i],
                8
            )
        )

        right_b.append(
            rc_utils.get_lidar_average_distance(
                scan,
                right_back_angle[i],
                8
            )
        )

        left_b.append(
            rc_utils.get_lidar_average_distance(
                scan,
                left_back_angle[i],
                8
            )
        )

    right_front = sum(right_f) / len(right_f)
    left_front  = sum(left_f) / len(left_f)

    right_back = sum(right_b) / len(right_b)
    left_back  = sum(left_b) / len(left_b)

    front_avg = (right_front + left_front) / 2

    if right_front >= left_front:

        if front_avg > left_back * 2.2:
            angle, speed = bothwallfollow2(scan)
            print(2)

        elif front_avg < left_back * 2.0:
            angle, speed = bothwallfollow1(scan)
            print(1)

    else:

        if front_avg > right_back * 2.2:
            angle, speed = bothwallfollow2(scan)
            print(2)

        elif front_avg < right_back * 2.0:
            angle, speed = bothwallfollow1(scan)
            print(1)

    if speed is None:
        speed = 1

    rc.drive.set_speed_angle(speed, angle)
