import numpy as np
import cv2


def show_matrix(grid, path = [], visited = [], start = None, end = None, waittime = 300):
    # --- Convert grid to image ---
    scale = 70  # make it bigger for visibility

    img = grid * 255  # 0->0, 1->255
    img = cv2.resize(img, (grid.shape[1]*scale, grid.shape[0]*scale),
                    interpolation=cv2.INTER_NEAREST)

    # Convert to color so we can draw colored path
    img_color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    # --- Draw the path ---
    for i in range(len(path) - 1):
        c1, r1 = path[i]
        c2, r2 = path[i + 1]

        pt1 = (c1 * scale + scale//2, r1 * scale + scale//2)
        pt2 = (c2 * scale + scale//2, r2 * scale + scale//2)

        cv2.line(img_color, pt1, pt2, (0, 0, 255), thickness=3)

    # Optional: draw points
    for c, r in path:
        center = (c * scale + scale//2, r * scale + scale//2)
        cv2.circle(img_color, center, 5, (255, 0, 0), -1)

    for c, r in visited:
        center = (c * scale + scale//2, r * scale + scale//2)
        cv2.circle(img_color, center, 5, (0, 0, 255), -1)
    
    if start:
        start = (start[0] * scale + scale//2, start[1] * scale + scale//2)
        cv2.circle(img_color, start, 5, (0, 255, 0), -1)
    if end:
        end = (end[0] * scale + scale//2, end[1] * scale + scale//2)
        cv2.circle(img_color, end, 5, (0, 255, 255), -1)

    # --- Show ---
    cv2.imshow("Path Visualization", img_color)
    cv2.waitKey(waittime)

if __name__ == "__main__":
    matrix = [[0, 0, 0, 1, 0, 1, 1, 0], [0, 1, 0, 1, 0, 1, 0, 0], [0, 1, 0, 0, 0, 1, 0, 1], [0, 1, 1, 1, 0, 1, 0, 0], [0, 0, 0, 1, 0, 1, 0, 0], [0, 0, 0, 1, 0, 1, 1, 0], [0, 1, 1, 1, 0, 0, 0, 0], [0, 1, 1, 1, 1, 1, 1, 1], [0, 0, 0, 0, 1, 1, 1, 1], [1, 0, 1, 0, 0, 0, 0, 1], [0, 0, 1, 1, 1, 1, 0, 1], [1, 1, 1, 0, 1, 0, 0, 0]]
    # Example binary matrix
    grid = np.array(matrix, dtype=np.uint8)

    # Path as list of (row, col)
    path = [(0, 0), (0, 1), (0, 2), (1, 2), (2, 2), (2, 3), (3, 3), (3, 4), (4, 4), (5, 4), (5, 5), (6, 5), (6, 6), (6, 7), (5, 7), (4, 7), (3, 7), (2, 7), (1, 7)]

    show_matrix(grid, waittime=0)