
import sys
sys.path.insert(1, "../../library")
import wallf
import line
import racecar_core
import numpy as np
import cv2 as cv
from rcdefine import rc
import racecar_utils as rc_utils
#rc = racecar_core.create_racecar()
#wallf.rc = rc

#line.rc = rc
line.CROP_FLOOR = ((360, 0), (rc.camera.get_height(), rc.camera.get_width()))

mode = "wall"

# ライン検出安定化用
line_detect_count = 0
line_lost_count = 0
MIN_CONTOUR_AREA = 30
CROP_FLOOR = ((360, 0), (rc.camera.get_height(), rc.camera.get_width()))

BLUE = ((100, 240, 240), (120, 255, 255))


def start():
    wallf.start(rc)
    line.start(rc)

    print("AUTO MODE START")


def update():
    global mode
    global line_detect_count
    global line_lost_count

    # ライン情報更新
    line.update_contour(rc)

    # =========================
    # wall → line
    # =========================
    image = rc.camera.get_color_image()

    if image is None:
        contour_center = None
        contour_area = 0
        return

    image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])

    contours = rc_utils.find_contours(image, BLUE[0], BLUE[1])
    contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)

    if contour is not None:

        contour_area = rc_utils.get_contour_area(contour)

        # ラインの一番下の点を使う
        points = contour[:, 0, :]
        lowest_point = points[np.argmin(points[:, 1])]

        contour_center = (
            int(lowest_point[1]),
            int(lowest_point[0])
        )

        cv.circle(
            image,
            (contour_center[1], contour_center[0]),
            8,
            (0, 255, 0),
            -1
        )
        mode = "line"
    else:
        contour_center = None
        contour_area = 0
        mode = "wall"
    '''
    if (
        mode == "wall"
        and line.contour_center is not None
        and line.contour_area > 30
        and 200 < line.contour_center[1] < 440
    ):

        line_detect_count += 1

    else:

        line_detect_count = 0

    if line_detect_count >= 3:

        mode = "line"
        line_detect_count = 0

        print("LINE MODE")


    # =========================
    # line → wall
    # =========================

    if (
        mode == "line"
        and (
            line.contour_center is None
            or line.contour_area < 20
        )
    ):

        line_lost_count += 1

    else:

        line_lost_count = 0

    if line_lost_count >= 5:

        mode = "wall"
        line_lost_count = 0

        print("WALL MODE")
'''

    # =========================
    # 実行
    # =========================

    if mode == "wall":
        scan = rc.lidar.get_samples()
        wallf.update(rc)

    elif mode == "line":
        image = rc.camera.get_color_image()
        line.update(rc)
        line.update_slow(rc)


def update_slow():
    pass
    #line.update_slow(rc)


if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
