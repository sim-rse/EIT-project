### orientatie (versie 2)

import cv2                          # OpenCV → camera en beeldverwerking
import numpy as np                  # numpy → wiskundige berekeningen
import cv2.aruco as aruco           # aruco → detecteren van ArUco markers

dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)  # laad standaard ArUco dictionary (type 4x4, 50 mogelijke markers)
detector = aruco.ArucoDetector(dictionary)                     # maak detector object met deze dictionary

def aruco_angle_from_x_axis(image, id_sought = 0):
    # image: BGR frame from cv2
    # marker_length: real marker size (any unit, only needed for pose)

    corners, ids, _ = detector.detectMarkers(image)   # detecteer markers in het beeld (geeft hoeken en IDs terug)
    

    if ids is None:                                   # als er geen markers gevonden zijn
        print("geen aruco gevonden")                  # foutmelding
        return 0                                     # geef 0 terug als hoek
    
    marker_dict = {id_[0]: corner for corner, id_ in zip(corners, ids)}  # maak dictionary: ID → bijhorende hoekpunten
    if id_sought not in marker_dict:                  # controleer of de gezochte marker aanwezig is
        print("geen aruco van juiste index gevonden !!")  # foutmelding
        return 0                                     # geef 0 terug

    pts = marker_dict[id_sought][0]                   # neem de hoekpunten van de gewenste marker
    v = pts[1] - pts[0]                              # vector langs bovenrand van marker (richting bepalen)
    angle = np.degrees(np.arctan2(v[1], v[0]))        # bereken hoek t.o.v. x-as in graden
    return angle                                     # geef hoek terug

if __name__ == "__main__":
    cap = cv2.VideoCapture(0)                        # start camera
    while True:                                      # oneindige lus
        _, color_image = cap.read()                  # lees frame van camera
        corners, ids, _ = detector.detectMarkers(color_image)  # detecteer markers opnieuw (voor tekenen)

        angle = aruco_angle_from_x_axis(color_image,3)  # bereken hoek van marker met ID = 3

        aruco.drawDetectedMarkers(color_image, corners, ids)  # teken kaders rond gedetecteerde markers
        cv2.putText(color_image, f"Angle: {int(angle)} deg", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)  # toon hoek op beeld
        cv2.imshow("markers", color_image)            # toon beeld met markers en hoek
        

        if cv2.waitKey(1) & 0xFF == ord('q'):         # stop als 'q' wordt ingedrukt
            break

    cap.release()                                     # sluit camera
    cv2.destroyAllWindows()                           # sluit alle vensters
