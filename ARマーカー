def ar_reading():
    image = rc.camera.get_color_image()
    def get_ar_markers(image):
    # ArUcoから生のARマーカーデータを収集する
        aruco_detector = cv.aruco.ArucoDetector(
        cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_250), 
        cv.aruco.DetectorParameters(),
        )
        aruco_data = aruco_detector.detectMarkers(image)
        
        # aruco_dataで見つかったARマーカーを表すARMarkerオブジェクトのリスト
        markers = []
            
        for i in range(len(aruco_data[0])):
            # TODO: aruco_dataの各マーカーについて、角の座標とIDを抽出して、
            # 角の座標を(row, col)のフォーマットに変更し、このデータでARMarkerオブジェクトを作成する(3.1節を参照)
            first_corners = aruco_data[0][i][0].astype(np.int32)


            first_corners = np.array(first_corners)
            first_corners = [[corner[1],corner[0]] for corner in first_corners]
            first_id = aruco_data[1][i][0]
            # TODO: 新しいマーカーをmarkersリストに追加する
            
            
            markers.append(ARMarker(first_id,first_corners))

            
        return markers

    class Orientation(Enum):
        UP = 0
        LEFT = 1
        DOWN = 2
        RIGHT = 3

    class ARMarker:
        
        def __init__(self, marker_id, marker_corners):
        
            self.__marker_id = marker_id
            self.__marker_corners = marker_corners

                    
        
            if self.__marker_corners[0][1] > self.__marker_corners[2][1]:
                    if self.__marker_corners[0][0] > self.__marker_corners[2][0]:
                        self.__orientation = Orientation.DOWN
                    else:
                        self.__orientation = Orientation.RIGHT
            else:
                    if self.__marker_corners[0][0] > self.__marker_corners[2][0]:
                        self.__orientation = Orientation.LEFT
                    else:
                        self.__orientation = Orientation.UP
            self.__greatest_area = 0
            self.__detected_color = "none"

            
        def detect_colors(self, image, potential_colors):

            marker_top, marker_left = marker.get_corners()[marker.get_orientation().value]
            marker_bottom, marker_right = marker.get_corners()[(marker.get_orientation().value + 2) % 4] 
            half_marker_height = (marker_bottom-marker_top)//2
            half_marker_width = (marker_right-marker_left)//2
            
            cropped_top_left = ((max(0,marker_top-half_marker_height)),(max(0,marker_left-half_marker_width)))
            cropped_bottom_right = ((min(image.shape[0],marker_bottom+half_marker_height))+1,(min(image.shape[1],marker_right+half_marker_width))+1)
            
            cropped_image= rc_utils.crop(image,cropped_top_left,cropped_bottom_right)
            marker_top, marker_left = marker.get_corners()[marker.get_orientation().value]
            marker_bottom, marker_right = marker.get_corners()[(marker.get_orientation().value + 2) % 4] 


        

            half_marker_height = (marker_bottom-marker_top)//2
            half_marker_width = (marker_right-marker_left)//2
            
            cropped_top_left = ((max(0,marker_top-half_marker_height)),(max(0,marker_left-half_marker_width)))
            cropped_bottom_right = ((rc_utils.clamp(marker_bottom+half_marker_height,0,479)),(rc_utils.clamp(marker_right+half_marker_width,0,639)))
        
            cropped_image= rc_utils.crop(image,cropped_top_left,cropped_bottom_right)
                
            for (hsv_lower, hsv_upper, color_name) in potential_colors:
                contours=  rc_utils.find_contours(cropped_image, hsv_lower, hsv_upper)
                largest_contour = rc_utils.get_largest_contour(contours)
            
                if largest_contour is not None:
                    contour_area = rc_utils.get_contour_area(largest_contour)
                
                    if contour_area > self.__greatest_area:
                        self.__greatest_area = contour_area
                        self.__detected_color = color_name
            return self.__detected_color

                
        def get_id(self):
            return self.__marker_id[0]

        
        def get_corners(self):
            return self.__marker_corners

        
        def get_orientation(self):
            return self.__orientation
        
        def get_color(self):
            return self.__detected_color
        def to_list(self):
            return [
            self.__marker_id,
            self.__marker_corners,
            self.__orientation,
            self.__detected_color
            ]
    ar_list = []
    markers = get_ar_markers(image)
    potential_colors = [
            ((160,0,0), (10, 255, 255), "red"),
            ((90, 160, 160), (120, 255, 255), "blue"),
            ((40, 50, 50), (80, 255, 255), "green")
            ]
    mark = 0
    for marker in markers:
        ar_list.append(marker.to_list())

    return ar_list 
