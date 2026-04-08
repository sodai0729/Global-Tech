def wall_following(): 
    global integral, prev_error,prev_prev_error
#　integralは使用しない

    scan = rc.lidar.get_samples()
# Liderが読み取る角度↓
    right_angle = [i for i in range(25,65,5)]
    left_angle = [i for i in range(335,295,-5) ]
    
#読み取とった角度に対応する距離のリストの作成
    right_distance = []
    left_distance = []

    for i in range(len(right_angle)):
        right_distance.append(rc_utils.get_lidar_average_distance(scan, right_angle[i], 8))
        left_distance.append(rc_utils.get_lidar_average_distance(scan, left_angle[i], 8))

    # 左右の平均距離
    right_avg = sum(right_distance) / len(right_distance)
    left_avg = sum(left_distance) / len(left_distance)
    a =1
    # 誤差
    error = right_avg - left_avg

    # PIDゲイン（調整必要！）
    Kp = 0.0155
    Ki = 0.0
    Kd = 0.75

    # 積分
    integral += error
    integral = rc_utils.clamp(integral,-10,10)

    # 微分
    derivative = error - prev_error

　　# エラーの値が徐々に大きくなっていくときの修正のプログラム、いらないかも
    if error != prev_error and prev_error != prev_error:
        if error >0:
            if error -prev_error >prev_error-prev_prev_error:
                a =(error - prev_error)/(prev_error-prev_prev_error)
        elif error <0:
            if prev_error-error > prev_prev_error-prev_error:
                a = (prev_error-error)/(prev_prev_error-prev_error)


    # PID出力
    angle = Kp * error*a + Ki * integral + Kd * derivative

    # クリップ
    angle = rc_utils.clamp(angle, -1, 1)
    prev_prev_error = prev_error
    prev_error = error
#シミュレーションではspeed=1で行った。実機の場合では  angleが大きい場合変える必要があるかも
    speed = 1
    return angle, speed
