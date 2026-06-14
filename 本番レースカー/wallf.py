
import sys


sys.path.insert(1, '../../library')
#import racecar_core


#rc = racecar_core.create_racecar()
from rcdefine import rc

import racecar_utils as rc_utils
import math
import actualdefine
#from define import bothwallfollow1

integral = 0
prev_error = 0
prev_prev_error = 0
target_distance = 60
A =0
prev_right=100
prev_left= 100
count = 0
def start(rc):
    global A,angle,integral,prev_error,prev_prev_error,target_distance,speed,mode
    integral = 0
    prev_error = 0
    prev_prev_error = 0
    target_distance = 60
    A = 180
    angle = 0
    speed = 0
    mode = "straight"
    print(">> Wall Following START")


def update(rc):
    global speed,angle,A,mode
    scan = rc.lidar.get_samples()
    #mode = actualdefine.Four_way_values(scan,A)
    mode,right,left,l,l = actualdefine.switch_wall_mode2(scan,A,mode)
    #mode = actualdefine.value_judge(scan,A)
    if mode == "straight":
        Kp,Kd,value,targetdistance = actualdefine.bothwallfollow1(scan,A)
        angle= actualdefine.AnglePIDcontrol(Kp,Kd,targetdistance,value)
        
    elif mode =="right_carve":
        Kp,Kd,value,targetdistance = actualdefine.bothwallfollow2(scan,A)
        angle= actualdefine.AnglePIDcontrol(Kp,Kd,targetdistance,value)
        angle =max(0.7,angle)
    
    if rc.controller.was_pressed(rc.controller.Button.A):
        A = 180
    if rc.controller.was_pressed(rc.controller.Button.B):
        A  = 0
    
    print(f"mode: {mode}, right: {right:.1f}, left: {left:.1f},angle:{angle},speed: {speed}")
    speed = -actualdefine.Speedcontrol(scan,angle,A)
    
    actualdefine.log(mode,right,left,angle)
    #print((right_front+left_front)/(right_back*2+0.0000000000001))
    #print(angle)
    #define.polar_plot(scan)

    

    '''
    print(
    f"RF={right_front:.1f} "
    f"LF={left_front:.1f} "
    f"RB={right_back:.1f} "f"LB={left_back:.1f}")
    '''
    rc.drive.set_speed_angle(speed, angle)
# [FUNCTION] update_slow() is similar to update() but is called once per second by
# default. It is especially useful for printing debug messages, since printing a 
# message every frame in update is computationally expensive and creates clutter



########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()




########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()
