# -*- coding: utf-8 -*-
"""
Created on Wed Mar  4 10:30:15 2026

@author: zahid
"""

import cv2 # deze laadt opencv
import numpy as np

# Camera starten
cap = cv2.VideoCapture(0)# we maken een camera obj aan , 0 staat voor eerste camera van computer

while True:
    ret, frame = cap.read()
    if not ret:
        break #als de camera faalt stop camera 

    # 1. Grijs maken
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)#basiskleuren zijn 3 matrices: rood, blauw en geel dus als we grijs nemen dan hebben we maar een matrix nodig 

    # 2. Blur (maakt beeld wazig opm ruis te verminderen)
    blur = cv2.GaussianBlur(gray, (5, 5), 0) #(5,5) is grootte van filter, de 0 is de camera die we gebruikt hadden 
#Kleine witte puntjes kunnen foutieve detectie geven, Blur maakt beeld stabieler.

    # 3. Threshold (Zet grijsbeeld om naar zwart-wit; zwarte tape wordt wit, vloer wordt zwart)
    _, thresh = cv2.threshold(blur, 100, 255, cv2.THRESH_BINARY_INV)
# cv2.threshold(bron, drempel, maxwaarde, type)
   
# 4.Bepaal hoogte en breedte van beeld en neem alleen de onderste helft (Region of Interest) voor lijndetectie)
    height, width = thresh.shape
    roi = thresh[int(height/2):height, :]

    # 5. Massamiddelpunt berekenen:Bereken het massamiddelpunt van de witte pixels in het geselecteerde gebied
    moments = cv2.moments(roi)

    if moments["m00"] != 0: # Als er een lijn is:
        cx = int(moments["m10"] / moments["m00"]) #Bereken het midden van de witte pixels (massamiddelpunt)


        # Visuele lijn tekenen
        cv2.line(frame, (cx, int(height*0.75)), (cx, height), (0, 0, 255), 2)#Teken een rode lijn op dat midden in het originele beeld


        # 6. Beslissing nemen
        #   Controleer of het midden links, rechts of in het midden zit en print de richting
        if cx < width / 2 - 20:
            print("LINKS")
        elif cx > width / 2 + 20:
            print("RECHTS")
        else:
            print("RECHT")
    else: #Als er geen lijn is, print "GEEN LIJN"
        print("GEEN LIJN")

    #Toon het originele beeld en het zwart-wit beeld
    cv2.imshow("Origineel", frame)
    cv2.imshow("Threshold", thresh)

    # Stoppen met q
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()