### orientatie (versie 2)

import cv2
import numpy as np
import cv2.aruco as aruco

dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
detector = aruco.ArucoDetector(dictionary)

def aruco_angle_from_x_axis(image, id_sought = 0):
    # image: BGR frame from cv2
    # marker_length: real marker size (any unit, only needed for pose)

    corners, ids, _ = detector.detectMarkers(image)
    

    if ids is None:
        print("geen aruco gevonden")
        return 0
    
    marker_dict = {id_[0]: corner for corner, id_ in zip(corners, ids)}
    if id_sought not in marker_dict:
        print("geen aruco van juiste index gevonden !!")
        return 0

    pts = marker_dict[id_sought][0]
    v = pts[1] - pts[0]  # top edge direction
    angle = np.degrees(np.arctan2(v[1], v[0]))
    return angle

if __name__ == "__main__":
    cap = cv2.VideoCapture(0)
    while True:
        _, color_image = cap.read()
        corners, ids, _ = detector.detectMarkers(color_image)

        angle = aruco_angle_from_x_axis(color_image)

        aruco.drawDetectedMarkers(color_image, corners, ids)
        cv2.putText(color_image, f"Angle: {int(angle)} deg", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
        cv2.imshow("markers", color_image)
        

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()