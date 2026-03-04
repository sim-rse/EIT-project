import cv2
import cv2.aruco as aruco
import numpy as np
img = cv2.imread("data/testarea.png")
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
detector = aruco.ArucoDetector(aruco_dict)

cap = cv2.VideoCapture(0) #camera id = 0

while True:

    corners, ids, _ = detector.detectMarkers(cap)

    print(f"corners: {corners}\nid's: {ids}")

    if ids is None or len(ids) < 4:
        print("Niet genoeg markers")

    marker_dict = {id_[0]: corner for corner, id_ in zip(corners, ids)}

    pts = np.array([
        marker_dict[1][0][0],  # TL
        marker_dict[2][0][1],  # TR
        marker_dict[3][0][2],  # BR
        marker_dict[4][0][3],  # BL
    ], dtype=np.float32)


    width = 600
    height = 400

    dst = np.array([
        [0, 0],
        [width, 0],
        [width, height],
        [0, height]
    ], dtype=np.float32)

    M = cv2.getPerspectiveTransform(pts, dst)
    warped = cv2.warpPerspective(cap, M, (width, height))


    aruco.drawDetectedMarkers(cap, corners, ids)
    cv2.imshow("markers", cap)
    cv2.imshow("warped", warped)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cv2.destroyAllWindows()