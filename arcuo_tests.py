import cv2
import cv2.aruco as aruco
import numpy as np
import time
from Realsense import RealSenseCamera

MODE = 'webcam'

#img = cv2.imread("data/testarea.png")
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
detector = aruco.ArucoDetector(aruco_dict)

def get_zone(image, zone_width = 8, zone_height = 12):
    corners, ids, _ = detector.detectMarkers(image)

    print(f"corners: {corners}\nid's: {ids}")
    

    if ids is None or len(ids) < 4:
        print("Niet genoeg markers")
        #raise ValueError(f"Niet genoeg markers gevonden: id's = {ids} corners = {corners}")
    else:
        marker_dict = {id_[0]: corner for corner, id_ in zip(corners, ids)}
        pts = np.array([
            marker_dict[1][0][0],  # TL
            marker_dict[2][0][1],  # TR
            marker_dict[3][0][2],  # BR
            marker_dict[4][0][3],  # BL
        ], dtype=np.float32)



        dst = np.array([
            [0, 0],
            [zone_width, 0],
            [zone_width, zone_height],
            [0, zone_height]
        ], dtype=np.float32)

        M = cv2.getPerspectiveTransform(pts, dst)
        warped = cv2.warpPerspective(image, M, (zone_width, zone_height))

        cv2.imshow("warped", warped)

    aruco.drawDetectedMarkers(image, corners, ids)
    cv2.imshow("markers", image)
    return 0

if __name__ == "__main__":
    if MODE == 'realsense':
        camera = RealSenseCamera()
        camera.start_async()
    elif MODE == 'webcam':
        cap = cv2.VideoCapture(0) #camera id = 0
    else:
        raise NameError('not existing mode selected, please check your spelling')

    while True:
        if MODE == 'realsense':
            frame = camera.get_latest_frame()
            color_image = frame.color_map
        else:
            _, color_image = cap.read()
        
        get_zone(color_image)
        

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        time.sleep(0.5)
    cap.release()
    cv2.destroyAllWindows()