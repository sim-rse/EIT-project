import cv2
import numpy as np

aruco = cv2.aruco

# Choose dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()

detector = aruco.ArucoDetector(aruco_dict, parameters)


def detect_marker(frame, target_id):
    corners, ids, _ = detector.detectMarkers(frame)

    if ids is None:
        return None

    for i, id_val in enumerate(ids.flatten()):
        if id_val == target_id:
            marker_corners = corners[i][0]  # 4 points
            return marker_corners

    return None

def get_center(corners):
    return np.mean(corners, axis=0)  # (x, y)

def get_orientation(corners):
    top_left = corners[0]
    top_right = corners[1]

    dx = top_right[0] - top_left[0]
    dy = top_right[1] - top_left[1]

    angle = np.arctan2(dy, dx)  # radians
    return angle

def pixel_to_cell(x, y, frame_w, frame_h, cols=12, rows=8):
    cell_w = frame_w / cols
    cell_h = frame_h / rows

    col = int(x // cell_w)
    row = int(y // cell_h)

    return (row, col)

path = [(0,0), (0,1), (0,2), ...]
current_index = 0

def process_frame(frame, path, current_index, target_id = 0):
    frame_h, frame_w, _ = frame.shape

    corners = detect_marker(frame, target_id)

    if corners is None:
        return current_index, "NO_MARKER"

    center = get_center(corners)
    angle = get_orientation(corners)

    cell = pixel_to_cell(center[0], center[1], frame_w, frame_h)

    target_cell = path[current_index]

    # CASE 1: reached target
    if cell == target_cell:
        current_index += 1

        if current_index < len(path):
            return current_index, "NEXT_INSTRUCTION"
        else:
            return current_index, "FINISHED"

    # CASE 2: still previous cell
    elif current_index > 0 and cell == path[current_index - 1]:
        return current_index, "KEEP_GOING"

    # CASE 3: wrong cell
    else:
        return current_index, "WRONG_CELL"