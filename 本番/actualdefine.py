
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
    enough_distance_to_the_front =350
    long_distance_to_turn = 280
    close_distance_to_turn = 350

    right_s_angle = [i for i in range(15+A,26+A,5)]
    left_s_angle = [i for i in range(335+A,346+A,5) ]
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
    print(right_avg,left_avg)
    #Kp = 0.001
    Kp = 0.0005
    Ki = 0.0
    Kd = 0.05
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

    
    Kp = 0.023
    Kd = 0.05
    targetdistance = right_avg
    value = left_avg
    return Kp,Kd,targetdistance,value

def Speedcontrol(scan,angle,A):
    right_distance =rc_utils.get_lidar_average_distance(scan,5+A, 8)
    left_distance = rc_utils.get_lidar_average_distance(scan,-5+A, 8)
    if right_distance < 450 and left_distance < 450:
        MAX_speed = 0.1
    else:
        MAX_speed = 1.0

    if angle < 0:
        angle = -angle
    
    speed = rc_utils.remap_range(angle,0,1,MAX_speed,0.1)
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


def switch_wall_mode1(scan,A,mode):
    transfer_stright = 30
    transfer_carve = 78
    value = 130
    sum_distance = 150
    swich_right_angle_to_stright=[i+A for i in range(transfer_stright,transfer_stright+11,5)] 
    swich_left_angle_to_stright=[i+A for i in range(360-transfer_stright,360-transfer_stright-11,-5)] 
    swich_right_angle_to_carve=[i+A for i in range(transfer_carve,transfer_carve+11,5)]
    swich_left_angle_to_carve=[i+A for i in range(360-transfer_carve,360-transfer_carve-11,-5)]

    swich_right_to_stright = []
    swich_left_to_stright = []
    swich_right_to_carve = []
    swich_left_to_carve = []

    for i in range(len(swich_right_angle_to_stright)):
        swich_right_to_stright.append(rc_utils.get_lidar_average_distance(scan, swich_right_angle_to_stright[i], 6))
        swich_left_to_stright.append(rc_utils.get_lidar_average_distance(scan, swich_left_angle_to_stright[i], 6))
        swich_right_to_carve.append(rc_utils.get_lidar_average_distance(scan, swich_right_angle_to_carve[i], 6))
        swich_left_to_carve.append(rc_utils.get_lidar_average_distance(scan, swich_left_angle_to_carve[i], 6))

    swich_right_stright = sum(swich_right_to_stright) / len(swich_right_to_stright)
    swich_left_stright = sum(swich_left_to_stright) / len(swich_left_to_stright)
    swich_right_carve = sum(swich_right_to_carve) / len(swich_right_to_carve)
    swich_left_carve = sum(swich_left_to_carve) / len(swich_left_to_carve)

    if mode == "stright":
        if swich_right_carve+swich_left_carve > sum_distance:
            mode = "carve"

    elif mode == "carve":

       if swich_right_carve < swich_left_carve:
           mode = "stright"
    return mode


def switch_wall_mode2(scan,A,mode):
    transfer_stright = 20
    transfer_carve = 78
    close_distance = 180
    far_distance = 380
    straight_far_distance = 330
    sum_distance = 280
    switch_right_angle_to_stright=[i+A for i in range(transfer_stright,transfer_stright+11,5)] 
    switch_left_angle_to_stright=[i+A for i in range(360-transfer_stright,360-transfer_stright-11,-5)] 
    switch_right_angle_to_carve=[i+A for i in range(transfer_carve,transfer_carve+11,5)]
    switch_left_angle_to_carve=[i+A for i in range(360-transfer_carve,360-transfer_carve-11,-5)]

    switch_right_to_stright = []
    switch_left_to_stright = []
    switch_right_to_carve = []
    switch_left_to_carve = []

    for i in range(len(switch_right_angle_to_stright)):
        switch_right_to_stright.append(rc_utils.get_lidar_average_distance(scan, switch_right_angle_to_stright[i], 6))
        switch_left_to_stright.append(rc_utils.get_lidar_average_distance(scan, switch_left_angle_to_stright[i], 6))
        switch_right_to_carve.append(rc_utils.get_lidar_average_distance(scan, switch_right_angle_to_carve[i], 6))
        switch_left_to_carve.append(rc_utils.get_lidar_average_distance(scan, switch_left_angle_to_carve[i], 6))

    switch_right_straight = sum(switch_right_to_stright) / len(switch_right_to_stright)
    switch_left_straight = sum(switch_left_to_stright) / len(switch_left_to_stright)
    switch_right_carve = sum(switch_right_to_carve) / len(switch_right_to_carve)
    switch_left_carve = sum(switch_left_to_carve) / len(switch_left_to_carve)

    if mode == "straight":
        if switch_right_carve > close_distance :
            mode = "right_carve"
        else:
            mode = "straight"
    elif mode == "right_carve":
        if  switch_right_straight  >straight_far_distance:
            mode = "straight"
        elif switch_right_carve + switch_left_carve < sum_distance:
            mode = "straight"
        else:
            mode = "right_carve"
    return mode,switch_right_straight,switch_left_straight,switch_right_carve,switch_left_carve

f = open("./logs.csv",mode="w")
f.write(",".join(["mode","right","left","angle","speed"])+"\n")
def log(*args):
    f.write(",".join([str(arg) for arg in args])+"\n")
