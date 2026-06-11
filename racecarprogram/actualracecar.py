import sys

sys.path.insert(1, '../../library')
import racecar_core
rc = racecar_core.create_racecar()


#rc = racecar_core.create_racecar()
import racecar_utils as rc_utils
import math
import racecardefine



def start():
    #ここであらかじめ指定する変数を記入
    global MIN_CONTOUR_AREA, CROP_FLOOR, BLUE, A, prev_error, prev_prev_error
    MIN_CONTOUR_AREA = 30
    CROP_FLOOR = ((360, 0), (rc.camera.get_height(), rc.camera.get_width()))

    BLUE = ((90, 50, 50), (120, 255, 255))
    A = 0
    prev_error = 0
    prev_prev_error = 0
    mode = True
    print(">>Racecar START")

def update():
    global MIN_CONTOUR_AREA, CROP_FLOOR, BLUE, A, prev_error, prev_prev_error,mode
    image = rc.camera.get_color_image()
    scan = rc.lidar.get_samples()
    followmode =racecardefine.wall_line_swich(image,MIN_CONTOUR_AREA,CROP_FLOOR,BLUE)

    if followmode == "line":
        Kp,Kd,targetdistance,value=racecardefine.linefollow(image,MIN_CONTOUR_AREA, CROP_FLOOR, BLUE)
        angle = racecardefine.AnglePIDcontrol(Kp,Kd,targetdistance,value)
    elif followmode == "wall":
        right_front,left_front,right_back,left_back = racecardefine.Four_way_values(scan,A)
        angle = racecardefine.bothwallfollow1(scan,A)
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

        if mode:
            Kp,Kd,value,targetdistance,r,l = racecardefine.bothwallfollow1(scan,A)
            angle= racecardefine.AnglePIDcontrol(Kp,Kd,targetdistance,value)
            print("mode1")
        else:
            Kp,Kd,value,targetdistance = racecardefine.bothwallfollow2(scan,A)
            angle= racecardefine.AnglePIDcontrol(Kp,Kd,targetdistance,value)
            print("mode2")

    speed = racecardefine.Speedcontrol(scan,angle,A)
    rc.drive.set_speed_angle(speed, angle)
def update_slow():
    None
    


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
