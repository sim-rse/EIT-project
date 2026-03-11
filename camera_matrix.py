# -*- coding: utf-8 -*-
"""
Created on Wed Mar  4 13:43:44 2026

@author: zahid
"""

import cv2
import numpy as np

# Camera starten
cap = cv2.VideoCapture(0)

# grootte van de doolhof-matrix instellen
matrix_rows = 500  # hoe groter = hoe meer resolutie
matrix_cols = 500

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # 1. Grijs maken
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 2. Blur voor minder ruis
    blur = cv2.GaussianBlur(gray, (5,5), 0)

    # 3. Threshold → zwart/wit
    _, thresh = cv2.threshold(blur, 100, 255, cv2.THRESH_BINARY_INV)

    # 4. GROOTSTE MATRIX: hele beeld
    #naar de juiste matrixgrootte gaan 
    small_thresh = cv2.resize(thresh, (matrix_cols, matrix_rows), interpolation=cv2.INTER_NEAREST)

    # Omzetten naar 0-1 matrix
    maze_matrix = small_thresh // 255

    print("Matrix:")
    print(maze_matrix)

    # 5.toon beelden
    cv2.imshow("Origineel", frame)
    cv2.imshow("Threshold", thresh)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()

cv2.destroyAllWindows()
