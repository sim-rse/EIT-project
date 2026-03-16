# -*- coding: utf-8 -*-
"""
Created on Wed Mar  4 13:43:44 2026

@author: zahid
"""

import cv2     #OpenCV library → camera en beeldverwerking
import numpy as np    #numpy → matrix en array bewerkingen

#Camera starten 
cap = cv2.VideoCapture(0)

#grootte van de doolhof-matrix instellen
matrix_rows = 500  # hoe groter = hoe meer resolutie
matrix_cols = 500

while True:                       #oneindige lus → blijft camera frames lezen
    ret, frame = cap.read()       #lees een frame van de camera
    if not ret:                   #als frame niet correct gelezen wordt
        break                     #stop de lus 

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  #zet kleurenbeeld om naar grijswaarden
    blur = cv2.GaussianBlur(gray, (5,5), 0) # vervaag het beeld licht om ruis te verminderen
    
    # pixels > 100 worden zwart en pixels < 100 worden wit (omgekeerde threshold)
    _, thresh = cv2.threshold(blur, 100, 255, cv2.THRESH_BINARY_INV) 

    # 4. GROOTSTE MATRIX: hele beeld
    #naar de juiste matrixgrootte gaan , verklein/vervorm het beeld naar een matrix van 500x500
    small_thresh = cv2.resize(thresh, (matrix_cols, matrix_rows), interpolation=cv2.INTER_NEAREST)

    # Omzetten naar 0-1 matrix
    # deel elke pixelwaarde door 255 → 0 blijft 0, 255 wordt 1 → zo krijg je een 0-1 matrix van het doolhof
    maze_matrix = small_thresh // 255  

    #print("Matrix:")
    #print(maze_matrix)

    # 5.toon beelden
    cv2.imshow("Origineel", frame)  # toont het originele camerabeeld
    cv2.imshow("Threshold", thresh) # toont het zwart/wit beeld na threshold

    if cv2.waitKey(1) & 0xFF == ord('q'): #als je op q indrukt venster sluite
        break

cap.release() # camera vrijgeven

cv2.destroyAllWindows()  # alle OpenCV vensters sluiten
