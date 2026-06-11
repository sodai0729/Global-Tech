import sys


sys.path.insert(1, '../../library')
#import racecar_core


#rc = racecar_core.create_racecar()
import racecar_utils as rc_utils
import math

prev_error=0
prev_prev_error=0

def AnglePIDcontrol(Kp,Kd,targetdistance,value):
    global prev_error,prev_prev_error
    #引数にPIDゲイン、目標値、実際の値をとる
    #角の値を返す
    error = value - targetdistance

    derivative = error - prev_error
    derivative2 = prev_error-prev_prev_error

    angle = Kp * error+ Kd * (derivative*0.3 +derivative2*0.7)
    #動きを滑らかにするために、1/60秒前の値と1/30秒前の値をとる
    angle = rc_utils.clamp(angle, -1, 1)
    prev_prev_error = prev_error
    prev_error = error
    
    return angle

def Four_way_values(scan,A):
    right_front_angle = [i for i in range(50+A,61+A,5)]
    left_front_angle = [i for i in range(300+A,311+A,5) ]
    right_back_angle = [i for i in range(120+A,131+A,5)]
    left_back_angle = [i for i in range(230+A,241+A,5) ]
   
    right_f = []
    left_f = []
    right_b = []
    left_b = []


    for i in range(len(right_front_angle)):
        right_f.append(rc_utils.get_lidar_average_distance(scan, right_front_angle[i], 8))
        left_f.append(rc_utils.get_lidar_average_distance(scan, left_front_angle[i], 8))
        right_b.append(rc_utils.get_lidar_average_distance(scan, right_back_angle[i], 8))
        left_b.append(rc_utils.get_lidar_average_distance(scan, left_back_angle[i], 8))

    # 平均距離
    right_front = sum(right_f) / len(right_f)
    left_front= sum(left_f) / len(left_f)
    left_back= sum(left_b) / len(left_b)
    right_back = sum(right_b) / len(right_b)

    return right_front,left_front,right_back,left_back
def bothwallfollow1(scan,A):
    right_angle = [15+A,20+A,25+A]
    left_angle = [335+A,34+A,345+A]
    

    right_distance = []
    left_distance = []

    for i in range(len(right_angle)):
        right_distance.append(rc_utils.get_lidar_average_distance(scan, right_angle[i], 8))
        left_distance.append(rc_utils.get_lidar_average_distance(scan, left_angle[i], 8))

    # 平均距離qa
    right_avg = sum(right_distance) / len(right_distance)
    left_avg = sum(left_distance) / len(left_distance)

    Kp = 0.0005
    Ki = 0.0
    Kd = 0.02
    targetdistance = right_avg
    value = left_avg
    return Kp,Kd,targetdistance,value

def bothwallfollow2(scan,A):
    right_angle = [i for i in range(80+A,91+A,5)]
    left_angle = [i for i in range(270+A,281+A,5) ]
    

    right_distance = []
    left_distance = []

    for i in range(len(right_angle)):
        right_distance.append(rc_utils.get_lidar_average_distance(scan, right_angle[i], 8))
        left_distance.append(rc_utils.get_lidar_average_distance(scan, left_angle[i], 8))


    right_avg = sum(right_distance) / len(right_distance)
    left_avg = sum(left_distance) / len(left_distance)

    
    Kp = 0.03
    Kd = 0.02
    targetdistance = right_avg
    value = left_avg
    return Kp,Kd,targetdistance,value

def Speedcontrol(scan,angle,A):
    front_distance =rc_utils.get_lidar_average_distance(scan,0+A, 8)

    if front_distance <=180:
        MAX_speed = 0.8
    else:
        MAX_speed = 1.0

    if angle < 0:
        angle = -angle
    
    speed = rc_utils.remap_range(angle,0,1,MAX_speed-0.7,0.1)
    speed = rc_utils.clamp(speed,0,1)


    return speed

def polar_plot(distances, max_distance=100, width=60, height=40):
    # ターミナル上の描画領域を初期化（空白で埋める)
    canvas = [[" " for _ in range(width)] for _ in range(height)]

    center_x = width // 2
    center_y = height // 2
    max_radius = min(center_x, center_y)
    max_distance = max_distance / 2

    for angle_deg, distance in enumerate(distances):
        # 無効な値をスキップ（例えば距離が0以下など）
        if distance <= 0:
            continue

        # 正規化して表示範囲に収める
        r = (distance / max_distance) * (max_radius - 1)

        # 極座標→デカルト座標
        angle_rad = math.radians(angle_deg)
        x = int(center_x + r * math.cos(angle_rad))
        y = int(center_y - r * math.sin(angle_rad) * 0.52)  # 上が小さいのでマイナス

        if 0 <= x < width and 0 <= y < height:
            canvas[y][x] = "*"

    # 描画（行単位で出力）
    for row in canvas:
        print("".join(row))
