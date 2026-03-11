# -*- coding: utf-8 -*-
"""
Created on Wed Mar 11 09:51:26 2026

@author: zahid
"""

import cv2
import numpy as np

# Camera starten
cap = cv2.VideoCapture(0)
matrix_rows = 500
matrix_cols = 500

while True:
    ret, frame = cap.read()                     # lees frame
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)          # kleur → grijs
    blur = cv2.GaussianBlur(gray, (5,5), 0)                # ruis verwijderen
    _, thresh = cv2.threshold(blur, 200, 255, cv2.THRESH_BINARY)  # detecteer witte tape

    # Contours detecteren
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    front_markers = []  # voorkant (2 witte tapes)
    back_marker = None  # achterkant (1 witte tape)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 50:  # filter kleine ruis
            x,y,w,h = cv2.boundingRect(cnt)
            cx = x + w//2
            cy = y + h//2

            if len(front_markers) < 2:
                front_markers.append((cx,cy))
            else:
                back_marker = (cx,cy)

            cv2.rectangle(frame, (x,y), (x+w, y+h), (0,255,0), 2)  # groene box

    # Oriëntatie berekenen
    if len(front_markers) == 2 and back_marker is not None:
        # midden van voorkant
        fx = (front_markers[0][0] + front_markers[1][0]) / 2
        fy = (front_markers[0][1] + front_markers[1][1]) / 2
        # vector richting achterkant
        dir_x = back_marker[0] - fx
        dir_y = back_marker[1] - fy
        angle = np.degrees(np.arctan2(dir_y, dir_x))
        # teken richtingpijl
        cv2.arrowedLine(frame, (int(fx), int(fy)), (int(back_marker[0]), int(back_marker[1])), (255,0,0), 2)
        cv2.putText(frame, f"Angle: {int(angle)} deg", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)

    # Beelden tonen
    cv2.imshow("Origineel", frame)
    cv2.imshow("Threshold", thresh)

    if cv2.waitKey(1) & 0xFF == ord('q'):       # stop bij 'q'
        break

cap.release()                                     # camera vrij
cv2.destroyAllWindows()                           # sluit vensters