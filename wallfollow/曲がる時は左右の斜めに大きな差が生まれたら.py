"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab G2 - LIDAR Wall Following (Perfect Symmetry & Speed 0.2 Turn Version)
"""

import sys
import cv2 as cv
import numpy as np

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# 変数の初期化
integral = 0.0
prev_error = 0.0
prev_prev_error = 0.0
speed = 0.3  
angle = 0.0

# モード2（直角カーブ専用割り込み）のための変数
is_mode2_active = False   
mode2_direction = 0.0     
mode2_counter = 0         

########################################################################################
# Functions
########################################################################################

def bothwallfollow1(scan):
    global integral, prev_error, prev_prev_error

    # 【LiDAR反転対策】直線用の本来の角度に180を足して、% 360 で0〜359の範囲に収める
    # 元の角度：右 20~50 / 左 310~340
    right_angle = [(deg + 180) % 360 for deg in range(20, 51, 5)]
    left_angle  = [(deg + 180) % 360 for deg in range(310, 341, 5)] 

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

    # 直線でのぐらつき（ハンチング）を抑えるゲイン
    Kp = 0.004   
    Ki = 0.0
    Kd = 0.022   

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

    return angle, 0.3


def start():
    print(">> Lab 4B - LIDAR Wall Following (Symmetrical Speed 0.2 Turn)")


def update():
    global speed, angle, is_mode2_active, mode2_direction, mode2_counter
    global integral, prev_error, prev_prev_error

    scan = rc.lidar.get_samples()

    # 【LiDAR反転対策】カーブ判定用の本来の角度（50〜60度）に180を足して、% 360 する
    # 元の角度：右 50~60 / 左 300~310
    right_front_angle = [(deg + 180) % 360 for deg in range(50, 61, 5)]    
    left_front_angle  = [(deg + 180) % 360 for deg in range(300, 311, 5)]  

    right_f = []
    left_f  = []

    for i in range(len(right_front_angle)):
        right_f.append(rc_utils.get_lidar_average_distance(scan, right_front_angle[i], 8))
        left_f.append(rc_utils.get_lidar_average_distance(scan, left_front_angle[i], 8))

    right_front = sum(right_f) / len(right_f)
    left_front  = sum(left_f) / len(left_f)
    
    # 今の瞬間の左右比率
    current_ratio = left_front / right_front if right_front != 0 else 1.0

    # -------------------------------------------------------------------------
    # 条件判定ロジック
    # -------------------------------------------------------------------------

    # 1. すでにモード2（カーブ）が発動中の場合
    if is_mode2_active:
        mode2_counter += 1  
        
        # 15フレーム経過するまでは比率を無視して全切り（0.2）を強制維持
        if mode2_counter > 15:
            # 15フレーム以上経ち、かつおよその正面（0.50〜2.00倍）を向いたら直線に戻る
            if 0.50 <= current_ratio <= 2.00:
                is_mode2_active = False
                mode2_counter = 0  
                
                # 直線に戻る瞬間にPIDのエラー履歴をリセット
                integral = 0.0
                prev_error = 0.0
                prev_prev_error = 0.0
                
                angle, speed = bothwallfollow1(scan)
            else:
                # 15フレーム過ぎてもまだ正面を向いていなければ全切りを維持
                angle = mode2_direction
                speed = 0.2  
        else:
            # 15フレーム未満のときは無条件ホールド
            angle = mode2_direction
            speed = 0.2  

    # 2. 通常走行中の場合
    else:
        # 左直角カーブのトリガー（1.30倍以上）
        if current_ratio >= 1.30:
            is_mode2_active = True
            mode2_counter = 1  
            mode2_direction = -1.0  
            angle = mode2_direction
            speed = 0.2  
            
        # 右直角カーブのトリガー（0.77倍以下）
        elif current_ratio <= 0.77:
            is_mode2_active = True
            mode2_counter = 1  
            mode2_direction = 1.0   
            angle = mode2_direction
            speed = 0.2  # ★ここを絶対に 0.2 に固定しました
            
        else:
            # 通常直線PID（スピード 0.3）
            angle, speed = bothwallfollow1(scan)
            speed = 0.3

    # ターミナル出力（処理軽量化のため angle と ratio のみ）
    print(f"angle: {angle:.2f}, ratio: {current_ratio:.2f}")

    rc.drive.set_speed_angle(speed, angle)


if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
