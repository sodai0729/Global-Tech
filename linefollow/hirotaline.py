MIN_CONTOUR_AREA = 30

BLUE = ((75, 0, 50), (120, 60, 130))
LOW_CROP_FLOOR = ((420, 0), (rc.camera.get_height(), rc.camera.get_width()))
Middle_CROP_FLOOR = ((340, 0), (400, rc.camera.get_width()))

def find_contour_center(image,crop_area,blue_hsv):
    global contour_center, contour_area
    image = rc_utils.crop(image, crop_area[0], crop_area[1])

    contours = rc_utils.find_contours(image, blue_hsv[0], blue_hsv[1])
    contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)

    if contour is not None:
        contour_center = rc_utils.get_contour_center(contour)
        contour_area = rc_utils.get_contour_area(contour)
    else:
        contour_center = None
        contour_area = 0

    return contour_center

def parts_of_line_PID(low_contour_center,mid_contour_center):
    
    if low_contour_center is None and mid_contour_center is None:
        linemode = "wall"
    elif low_contour_center is None and mid_contour_center is not None:
        linemode = "prepare"
    elif low_contour_center is not None and mid_contour_center is None:
        linemode = "wall"
    else:
        ''''
        if low_contour_center[1] <rightlimitplace and mid_contour_center[1] > rightlimitplace:
            linemode = "difference_line"
        elif low_contour_center[1] > leftlimitplace and mid_contour_center[1] < leftlimitplace:
            linemode = "difference_line"
        else:
            linemode = "nomal_line"
            '''
        linemode = "line"
    return linemode

def define_PID(mid_contour_center):
    Kp = 0.1
    Kd = 0.6
    width = 639

    error = rc_utils.remap_range(mid_contour_center[1],0,width,-1,1)
    return Kp,Kd,0,error
