import sys


sys.path.insert(1, '../../library')
#import racecar_core


#rc = racecar_core.create_racecar()
import racecar_utils as rc_utils
import math

prev_error=0
prev_prev_error=0
prev_right=100
prev_left=100
count = 11
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

def entire_Four_way_values(scan,A):
    right_front_angle = [i for i in range(40+A,61+A,5)]
    left_front_angle = [i for i in range(300+A,321+A,5) ]
    right_back_angle = [i for i in range(100+A,141+A,5)]
    left_back_angle = [i for i in range(220+A,261+A,5) ]
   
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

    if right_front >= left_front:
        if (right_front+left_front)/2> left_back*1.7:
            mode =False
        elif (right_front+left_front)/2< left_back*1.5:
            mode = True
            

    else:
        if (right_front+left_front)/2 > right_back*1.7:
            mode =  False
        elif (right_front+left_front)/2 < right_back*1.5:
            mode = True

    return mode

def forward_four_way_values(scan,A):
    allowable_difference_of_sharp = 1.05
    enough_distance_to_the_front =300
    long_distance_to_turn = 280
    close_distance_to_turn = 380

    right_s_angle = [i for i in range(15+A,26+A,5)]
    left_s_angle = [i for i in range(345+A,356+A,5) ]
    right_angle = [i for i in range(45+A,61+A,5)]
    left_angle = [i for i in range(300+A,316+A,5)]

    right_s = []
    left_s = []
    right_a = []
    left_a= []

    for i in range(len(right_s_angle)):
        
        right_s.append(rc_utils.get_lidar_average_distance(scan, right_s_angle[i], 8))
        left_s.append(rc_utils.get_lidar_average_distance(scan, left_s_angle[i], 8))
        right_a.append(rc_utils.get_lidar_average_distance(scan, right_angle[i], 8))
        left_a.append(rc_utils.get_lidar_average_distance(scan, left_angle[i], 8))

    # 平均距離
    right_sharp = sum(right_s) / len(right_s)
    left_sharp= sum(left_s) / len(left_s)
    left= sum(left_a) / len(left_a)
    right = sum(right_a) / len(right_a)
    if right_sharp is None or left_sharp is None:
        mode = "straight"
    else:
        if right_sharp < enough_distance_to_the_front or left_sharp < enough_distance_to_the_front:
            if right > long_distance_to_turn and left < close_distance_to_turn:
                mode = "carve"
            elif right < close_distance_to_turn and left > long_distance_to_turn:
                mode = "carve"
            else:
                mode = "straight"
        else:
            mode = "straight"

    return mode,right,left

def value_judge(scan,A):
    global prev_left,prev_right,count
    right_angle = [i for i in range(45+A,51+A,5)]
    left_angle = [i for i in range(310+A,316+A,5)]

    right_a = []
    left_a= []

    for i in range(len(right_angle)):
        right_a.append(rc_utils.get_lidar_average_distance(scan, right_angle[i], 8))
        left_a.append(rc_utils.get_lidar_average_distance(scan, left_angle[i], 8))

    # 平均距離
    left= sum(left_a) / len(left_a)
    right = sum(right_a) / len(right_a)
    if right > prev_right*1.4 or left > prev_left*1.4:
        count +=1
        mode = "carve"
    else:
        if count >10:
            mode = "straight"
            count = 0
        else:
            mode = "carve"
        
    prev_left =left
    prev_right = right
    return mode
def bothwallfollow1(scan,A):
    right_angle = [i for i in range(30+A,41+A,5)]
    left_angle = [i for i in range(320+A,331+A,5) ]
    

    right_distance = []
    left_distance = []

    for i in range(len(right_angle)):
        right_distance.append(rc_utils.get_lidar_average_distance(scan, right_angle[i], 8)*abs(math.cos(math.radians(i))))
        left_distance.append(rc_utils.get_lidar_average_distance(scan, left_angle[i], 8)*abs(math.cos(math.radians(i))))

    # 平均距離qa
    right_avg = sum(right_distance) / len(right_distance)
    left_avg = sum(left_distance) / len(left_distance)
    Kp = 0.0005
    #Kp = 0.02
    Ki = 0.0
    Kd = 0.005
    #Kd = 0.008
    targetdistance = right_avg
    value = left_avg
    return Kp,Kd,targetdistance,value

def bothwallfollow2(scan,A):
    right_angle = [i for i in range(50+A,71+A,5)]
    left_angle = [i for i in range(290+A,311+A,5) ]
    

    right_distance = []
    left_distance = []

    for i in range(len(right_angle)):
        right_distance.append(rc_utils.get_lidar_average_distance(scan, right_angle[i], 8))
        left_distance.append(rc_utils.get_lidar_average_distance(scan, left_angle[i], 8))


    right_avg = sum(right_distance) / len(right_distance)
    left_avg = sum(left_distance) / len(left_distance)

    
    Kp = 0.02
    Kd = 0.05
    targetdistance = right_avg
    value = left_avg
    return Kp,Kd,targetdistance,value

def Speedcontrol(scan,angle,A):
    front_distance =rc_utils.get_lidar_average_distance(scan,0+A, 8)
    a =0
    if front_distance <=180:
        MAX_speed = 0.6
    else:
        MAX_speed = 1.0

    if angle < 0:
        angle = -angle
    
    speed = rc_utils.remap_range(angle,0,1,MAX_speed-a,0.1)
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
