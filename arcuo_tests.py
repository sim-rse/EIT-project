import cv2
import cv2.aruco as aruco
import numpy as np
import time
from Realsense import RealSenseCamera
import matplotlib.pyplot as plt

MODE = 'picture'

#img = cv2.imread("data/testarea.png")
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
detector = aruco.ArucoDetector(aruco_dict)

def get_zone(image, zone_width = 8*10, zone_height = 12*10):
    corners, ids, _ = detector.detectMarkers(image)

    print(f"corners: {corners}\nid's: {ids}")
    
    def check_ids(ids, ids_to_look_for):
        treshhold = len(ids_to_look_for)
        already_found = []
        for id in ids:
            if id in ids_to_look_for and id not in already_found:
                already_found.append(id)
        return len(already_found)==treshhold
    
    if ids is None or not check_ids(ids,[1,2,3,4]):
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
    image = cv2.resize(image, (zone_width*3, zone_height*3))
    cv2.imshow("markers", image)
    return warped

def to_matrix(img, target_h = 12, target_w = 8):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    h, w = gray.shape

    block_h = h // target_h
    block_w = w // target_w

    # Crop so blocks fit perfectly
    gray = gray[:block_h * target_h, :block_w * target_w]

    # Reshape to blocks
    blocks = gray.reshape(target_h, block_h, target_w, block_w)

    # Average each block
    means = blocks.mean(axis=(1, 3))

    # Convert to binary
    bw = (means > 127).astype(int)

    #print(bw)
    return bw

def to_matrix2(img, target_h = 12, target_w = 8):

    # Convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Resize to target resolution
    small = cv2.resize(gray, (target_w, target_h), interpolation=cv2.INTER_AREA)

    # Threshold to get black/white
    _, bw = cv2.threshold(small, 127, 1, cv2.THRESH_BINARY)

    return bw

if __name__ == "__main__":
    if MODE == 'realsense':
        camera = RealSenseCamera()
        camera.start_async()
    elif MODE == 'webcam':
        cap = cv2.VideoCapture(0) #camera id = 0
    elif MODE =='picture':
        img = cv2.imread("data/labyrinthe_aruco_perspective.png")
    else:
        raise NameError('not existing mode selected, please check your spelling')

    while True:
        if MODE == 'realsense':
            frame = camera.get_latest_frame()
            color_image = frame.color_map
        elif MODE == 'picture':
            color_image = img
        else:
            _, color_image = cap.read()
        
        warped = get_zone(color_image)
        array = to_matrix2(warped)
        print(array)

        plt.imshow(array, cmap="gray")
        plt.show()
        if cv2.waitKey() & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()