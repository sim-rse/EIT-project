# -*- coding: utf-8 -*-
"""
MAIN - Motorauto doolhof navigatie met spiersensor en laser beveiliging
========================================================================
Gebaseerd op / combineert:
  - camera_matrix.py  → doolhof inlezen via camera (zwart = pad, wit = muur)
  - pad_herkenning.py → BFS kortste pad vinden, foute wegen worden automatisch vermeden
  - orientatie.py     → oriëntatie van de auto bepalen via witte tape markers
  - spier_enkel_notch → spiersensor met notchfilter, baseline, spierspanning detectie

Doolhof fysieke afmetingen:
  - 12 kolommen x 8 rijen cellen
  - korte zijde (8 cellen) = 2 m  → 1 cel = 0.25 m
  - lange zijde (12 cellen) = 3 m → 1 cel = 0.25 m
  - Ingang: linksonder  (rij 7, kolom 0)  pijl omhoog
  - Uitgang: rechtsboven (rij 0, kolom 11) pijl naar rechts

Laser beveiliging:
  - GL5537 LDR sensor + groene laser (532 nm)
  - Spanningsverdeler met 2 kΩ weerstand
  - Geen lasercontact: ~2.1 V over sensor  → auto mag NIET rijden
  - Met lasercontact:  ~0.8 V over sensor  → auto mag WEL rijden
  - Threshold: 1.4 V (midden tussen 0.8 en 2.1)
  - Laser is aangesloten op een ADC-pin van de microcontroller
"""

# ─────────────────────────────────────────────────────────────────────────────
# IMPORTS  (zelfde volgorde als in de bestaande files)
# ─────────────────────────────────────────────────────────────────────────────
from statistics import median       # mediaan voor baseline berekening (zelfde als spier_enkel_notch)

import serial                       # seriële communicatie met microcontroller
import numpy as np                  # matrix bewerkingen
from scipy import signal            # digitale filters (notchfilter)
import matplotlib.pyplot as plt     # grafieken plotten bij afsluiten (zelfde als spier_enkel_notch)
import cv2                          # camera en beeldverwerking (zelfde als camera_matrix / orientatie)
from collections import deque       # wachtrij voor BFS (zelfde als pad_herkenning)
import time                         # tijdsmeting voor snelheidsregeling
import cv2.aruco as aruco
import queue

from mazeshower import show_matrix
from miscelaneous import *
from frameprocessor import process_frame

# ─────────────────────────────────────────────────────────────────────────────
# MOTOR FUNCTIES
# ─────────────────────────────────────────────────────────────────────────────
# TODO: vervang de inhoud van elke functie door de echte motoraansturing
# De signatuur (naam + parameter) blijft hetzelfde, enkel de inhoud verandert
# speed = percentage van maximaal motorvermogen (0 tot 100)

MAXPWMUINT16 = 498 #dit is de PWM waarde die voor max dutycycle zorgt, hogere waardes geven gaan de snelheid niet verhogen

control_mode = "muscle" # Voor backup met spacebar

def PWM_left(dutycycle, forwards = True):       #geeft
    if forwards:
        ser_drive.write(f'l{int(dutycycle*MAXPWMUINT16)}\n'.encode('ascii'))
    else:
        ser_drive.write(f'a{int(dutycycle*MAXPWMUINT16)}\n'.encode('ascii'))
        

def PWM_right(dutycycle, forwards = True):
    if forwards:
        ser_drive.write(f'r{int(dutycycle*MAXPWMUINT16)}\n'.encode('ascii'))
    else:
        ser_drive.write(f'b{int(dutycycle*MAXPWMUINT16)}\n'.encode('ascii'))

def move_forward(speed):
    """Rijd rechtdoor aan de opgegeven snelheid (0-100 %)."""
    PWM_left(speed, True)
    PWM_right(speed, True)
    print(f"[MOTOR] Vooruit | snelheid={speed:.1f}%")
    # TODO: bijv. motor_links.forward(speed) en motor_rechts.forward(speed)

def move_backward(speed):
    """Rijd achteruit aan de opgegeven snelheid (0-100 %)."""
    PWM_left(speed, False)
    PWM_right(speed, False)
    print(f"[MOTOR] Achteruit | snelheid={speed:.1f}%")
    # TODO: bijv. motor_links.backward(speed) en motor_rechts.backward(speed)

def ROTSPEED(dutycycle):    #lineaire regressie om de hoeksnelheid van de draaing te berekenen afhankelijk van de dutycycle
    return ...

def turn_left(angle, dutycycle):             
    PWM_left(dutycycle, False)                     #hoe trager je draait hoe preciezer je kan zijn Note: deze functie zal mss in mplab moeten geimplementeerd worden voor betere preciesie
    PWM_right(dutycycle, True)
    time.sleep(angle/ROTSPEED(dutycycle))

def turn_right(angle, dutycycle):
    PWM_left(dutycycle)
    PWM_right(dutycycle, False)
    time.sleep(angle/ROTSPEED(dutycycle))


def turn_left_90(speed):
    """Draai 90° naar links aan de opgegeven snelheid (0-100 %)."""
    turn_left(90,speed)
    print(f"[MOTOR] Draai links 90° | snelheid={speed:.1f}%")
    # TODO: bijv. motor_links.backward(speed) en motor_rechts.forward(speed) + timing

def turn_right_90(speed):
    """Draai 90° naar rechts aan de opgegeven snelheid (0-100 %)."""
    turn_right(90,speed)
    print(f"[MOTOR] Draai rechts 90° | snelheid={speed:.1f}%")
    # TODO: bijv. motor_links.forward(speed) en motor_rechts.backward(speed) + timing

def stop_motors():
    """Stop alle motoren onmiddellijk."""
    move_forward(0)
    global huidige_snelheid
    huidige_snelheid = 0
    print("[MOTOR] Stop")
    # TODO: bijv. motor_links.stop() en motor_rechts.stop()

# ─────────────────────────────────────────────────────────────────────────────
# CONSTANTEN & INSTELLINGEN
# ─────────────────────────────────────────────────────────────────────────────

TESTMODE = False
UART_ENABLED = False

# ── Doolhof afmetingen (uit de opgave) ───────────────────────────────────────
matrix_rows = 8                 # aantal rijen in het doolhof (korte zijde = 2 m)
matrix_cols = 12                # aantal kolommen in het doolhof (lange zijde = 3 m)
START       = (0, 11)            # ingang: linksonder, pijl wijst omhoog             coordinatensysteem (y,x) met origin topleft
END         = (7, 1)           # uitgang: rechtsboven, pijl wijst naar rechts       (vraag mij niet waarom ma als je x en y omwisselt warkt het anders wordt het ergensgeswitchd. heb geen tijd om het te fixen in geval een assistent dit leest)

# ── Seriële poort (zelfde als spier_enkel_notch) ─────────────────────────────
BAUD          = 444444          # baudrate van de microcontroller
PORT          = 'COM6'          # seriële poort voor spier en laser sensoren → aanpassen indien nodig
PORT_DRIVE    = 'COM5'          # seriële poort voor controller op de wagen → ook aanpassen indien nodig
printsnelheid = 10              # Hz: aantal samples per seconde van de microcontroller
FS            = int(1000 / printsnelheid)  # sample frequentie in Hz (= 100 Hz)

# ── Spiersensor instellingen (zelfde als spier_enkel_notch) ──────────────────
lines_per_gemiddelde = 50       # aantal samples per gemiddelde voor grafiek
check_lines          = 50       # aantal samples per venster voor spierspanning detectie
drempel              = 50       # afwijking t.o.v. baseline om spier als gespannen te beschouwen

# ── Laser / LDR beveiliging ───────────────────────────────────────────────────
# GL5537 LDR met 2 kΩ spanningsverdeler, groene laser 532 nm
# Geen contact: ~2.1 V | Met contact: ~0.8 V
# ADC van microcontroller: 10 bit → 0-1023 overeenkomend met 0-5 V
# Threshold voltage = 1.4 V (midden tussen 0.8 en 2.1)
# Threshold in ADC waarden = (1.4 / 5.0) * 1023 ≈ 287
LASER_THRESHOLD_VOLTAGE = 1.4   # V: grens tussen "contact" en "geen contact"
LASER_ADC_MAX           = 1023  # maximale ADC waarde (10 bit)
LASER_VCC               = 5.0   # voedingsspanning microcontroller in volt
LASER_THRESHOLD_ADC     = 300
# Onder threshold → laser raakt sensor → lage weerstand → lage spanning → auto mag rijden
# Boven threshold → geen lasercontact → hoge weerstand → hoge spanning → auto mag NIET rijden

# ── Snelheidsregeling ─────────────────────────────────────────────────────────
SPEED_MIN         = 20.0        # minimale rijsnelheid (%) zodra spier gespannen is
SPEED_MAX         = 80.0        # maximale rijsnelheid (%)
SPEED_RAMP_UP     = 2.0         # snelheidstoename per seconde bij spierspanning (geleidelijk optrekken)
SPEED_RAMP_DOWN   = 5.0         # snelheidsafname per seconde bij loslaten (geleidelijk remmen)
MAX_SPEED_TO_TURN = 25.0        # auto moet trager zijn dan dit voordat hij mag draaien

# ─────────────────────────────────────────────────────────────────────────────
# VARIABELEN  (zelfde structuur als spier_enkel_notch, nu als globale variabelen)
# ─────────────────────────────────────────────────────────────────────────────

# ── Spiersensor lijsten (zelfde namen als spier_enkel_notch) ──────────────────
lijst_raw                = []   # alle ruwe ADC-waarden van de spiersensor
lijst_filtered           = []   # alle gefilterde waarden na notchfilter
lijst_filtered_2         = []   # absolute waarden van gefilterd signaal (voor plot)
lijst_filtered_intervals = []   # niet gebruikt, behouden voor consistentie met origineel

# ── Baseline variabelen (zelfde als spier_enkel_notch) ───────────────────────
baseline         = None         # mediaan van de eerste 5 seconden → rustwaarde
baseline_check   = []           # tijdelijke lijst om baseline te berekenen
baseline_teller  = 0            # telt hoeveel samples al voor baseline verzameld zijn
baseline_done    = False        # True zodra baseline berekend is

# ── Spierspanning detectie (zelfde als spier_enkel_notch) ────────────────────
teller           = 0            # teller binnen huidig venster van check_lines samples
gemiddelde_lijst = []           # samples in huidig venster voor gemiddelde berekening
glob_gemiddelden = []           # lijst van alle vorige gemiddelden (niet gebruikt, behouden)
spier_gespannen  = False        # True als spier momenteel gespannen is
spier_lijst      = []           # historiek van "True"/"False" strings per venster

# ── Snelheidsregeling ─────────────────────────────────────────────────────────
current_speed    = 0.0          # huidige rijsnelheid in % (stijgt/daalt geleidelijk)
last_time        = time.time()  # tijdstip van laatste snelheidsupdate

# ── Notchfilter initialisatie (zelfde als spier_enkel_notch) ─────────────────
b_notch, a_notch = signal.iirnotch(50.0, 30.0, FS)  # notchfilter op 50 Hz, Q-factor 30
zi_notch = signal.lfilter_zi(b_notch, a_notch) * 0   # beginwaarden filter (= 0 bij start)

# ── Aruco initialisatie ──────────────────────────────────────────────────────
dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
detector = aruco.ArucoDetector(dictionary)

# ─────────────────────────────────────────────────────────────────────────────
# DOOLHOF HERKRNNINGS FUNCTIEs  (overgenomen uit aruco_tests.py)
# ─────────────────────────────────────────────────────────────────────────────

def get_zone(image, zone_width = 8*30, zone_height = 12*30):
    corners, ids, _ = detector.detectMarkers(image)

    #print(f"corners: {corners}\nid's: {ids}")
    
    def check_ids(ids, ids_to_look_for):
        treshhold = len(ids_to_look_for)
        already_found = []
        for id in ids:
            if id in ids_to_look_for and id not in already_found:
                already_found.append(id)
        return len(already_found)==treshhold
    
    if ids is None or not check_ids(ids,[1,2,3,4]):
        print("Niet genoeg markers")
        return False, None
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

        #cv2.imshow("warped", warped)

    aruco.drawDetectedMarkers(image, corners, ids)
    image = cv2.resize(image, (zone_width*3, zone_height*3))
    cv2.imshow("markers", image)
    return True, warped

def to_matrix(img, target_h = 12, target_w = 8, threshold = 127):
    if len(img.shape) == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img  # already grayscale

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
    bw = (means > threshold).astype(int)

    #print(bw)
    return bw.tolist()

# ─────────────────────────────────────────────────────────────────────────────
# BFS FUNCTIE  (exact overgenomen uit pad_herkenning.py)
# ─────────────────────────────────────────────────────────────────────────────


def bfs(maze, start, end, allowed_color = 0):
    """
    BFS pathfinding (= breadth first search).
    Zoekt het kortste pad van start naar end in het doolhof.
    Foute wegen (doodlopende paden) worden automatisch vermeden
    omdat BFS altijd de kortste route uitkiest.
    maze[r][c] == 0 betekent vrij pad, == 1 betekent muur.
    """
    maze = np.array(maze, dtype=np.uint8)
    #maze = maze.transpose()
    rows, cols = len(maze), len(maze[0])    # afmetingen van de maze matrix ophalen
    queue = deque()                          # lege wachtrij aanmaken
    queue.append((start, [start]))           # beginpositie + pad tot nu toe toevoegen
    visited = set()                          # verzameling van al bezochte cellen
    visited.add(start)                       # beginpositie meteen markeren als bezocht
    directions = [(1,0),(-1,0),(0,1),(0,-1)] # onder, boven, rechts, links (zelfde als pad_herkenning)

    while queue:                             # blijf zoeken zolang er cellen in de wachtrij zitten
        (x,y), path = queue.popleft()        # volgende cel uit wachtrij halen
        if (x,y) == end:                     # doelcel bereikt → pad teruggeven
            print('oplossing gevonden!!')
            return path
        for dx,dy in directions:             # alle 4 buren bekijken
            nx, ny = x+dx, y+dy              # coördinaten van de buurcel berekenen
            if 0<=nx<cols and 0<=ny<rows:    # controleer of buurcel binnen het grid valt
                if maze[ny,nx]==allowed_color and (nx,ny) not in visited:  # vrij pad en nog niet bezocht
                    visited.add((nx,ny))     # markeer als bezocht zodat we niet terugkeren
                    queue.append(((nx,ny), path + [(nx,ny)]))   # voeg toe aan wachtrij met uitgebreid pad
                    show_matrix(maze, path, visited, start, end, waittime=5)
    cv2.destroyAllWindows()
    return None                              # geen pad gevonden (zou niet mogen bij correct doolhof)

def convert_to_path(my_node):
    my_list = []
    while True:
        my_list.append(my_node['pos'])
        if my_node['parent'] == None:
            return my_list
        my_node = my_node['parent']

def BFS_smart(maze, start, end, allowed_color = 0):
    traveled =[]
    start_state = {'pos':start,'parent':None}
    rows, cols = len(maze), len(maze[0])
    q = queue.Queue()
    q.put(start_state)
    iteration = 0

    show_matrix(np.array(maze, dtype=np.uint8), start=start, end=end, waittime=5)
    while q.empty() == False:
        state = q.get()
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        iteration += 1
        print(f"\riteration {iteration}", end="")
        for zet in directions:
            new_pos = (state['pos'][0]+zet[0],state['pos'][1]+zet[1])
            if new_pos not in traveled and 0<=new_pos[0]<rows and 0<=new_pos[1]<cols:
                if maze[new_pos[0]][new_pos[1]] == allowed_color:
                    new_state = {'pos':new_pos,'parent':state}
                    if new_pos == end :
                        print("Oplossing gevonden!!!")
                        path = convert_to_path(new_state)
                        return path
                    else: 
                        q.put(new_state)
                        traveled.append(new_pos)
                        path = convert_to_path(new_state)
                        show_matrix(np.array(maze, dtype=np.uint8), path, start=start, end=end, waittime=5)
    return None
# ─────────────────────────────────────────────────────────────────────────────
# ORIËNTATIE FUNCTIES  (gebaseerd op orientatie.py)
# ─────────────────────────────────────────────────────────────────────────────

def bepaal_orientatie(frame, id_sought = 0):
    """
    Bepaalt de oriëntatie van de auto op basis van witte tape markers.
    De auto heeft 2 witte tapes aan de voorkant en 1 aan de achterkant.
    Gebaseerd op orientatie.py: gebruikt het MIDDEN van de 2 voorkant-markers
    als referentiepunt, en berekent de hoek naar de achterkant.
    Geeft (hoek_in_graden, frame_met_annotaties) terug, of (None, frame) bij fout.
    """
    # image: BGR frame from cv2
    # marker_length: real marker size (any unit, only needed for pose)

    corners, ids, _ = detector.detectMarkers(frame)
    

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

    aruco.drawDetectedMarkers(frame, corners, ids)
    cv2.putText(frame, f"Angle: {int(angle)} deg", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)

    return angle, frame


def bereken_benodigde_draai(huidige_hoek, gewenste_richting):
    """
    Berekent hoeveel en in welke richting gedraaid moet worden.
    gewenste_richting is een (dr, dc) tuple (rijrichting van het BFS-pad).
    Geeft 'forward', 'turn_left', 'turn_right' of 'turn_back' terug.
    """
    # Zet gewenste rijrichting om naar een doelhoek in graden
    # In OpenCV: y-as wijst naar beneden, x-as naar rechts
    # arctan2(dy, dx): omhoog = -90°, omlaag = +90°, rechts = 0°, links = ±180°
    dr, dc = gewenste_richting
    doel_hoek = np.degrees(np.arctan2(dr, dc))  # hoek die overeenkomt met de gewenste richting

    # Bereken het verschil tussen huidige en gewenste hoek
    verschil = doel_hoek - huidige_hoek         # hoeveel graden moet er gedraaid worden

    # Normaliseer naar het bereik [-180, 180] graden
    verschil = (verschil + 180) % 360 - 180

    # Bepaal welke draaibeweging nodig is op basis van het verschil
    if abs(verschil) < 30:                      # minder dan 30° verschil → rechtdoor
        return 'forward'
    elif abs(verschil) > 150:                   # meer dan 150° verschil → 180° keren
        return 'turn_back'
    elif verschil > 0:                          # positief verschil → rechts draaien
        return 'turn_right'
    else:                                       # negatief verschil → links draaien
        return 'turn_left'


# ─────────────────────────────────────────────────────────────────────────────
# LASER FUNCTIE
# ─────────────────────────────────────────────────────────────────────────────

def lees_laser_status(ser):
    """
    Leest de ADC-waarde van de GL5537 LDR laser-sensor via de seriële poort.
    De microcontroller stuurt een lijn met 'laser: <ADC_waarde>'.

    Spanningsverdeler met 2 kΩ weerstand en 5 V voeding:
      - Geen lasercontact: hoge LDR weerstand → hoge spanning (~2.1 V) → hoge ADC waarde
      - Met lasercontact:  lage LDR weerstand  → lage spanning  (~0.8 V) → lage ADC waarde

    Threshold: 1.4 V → ADC waarde ≈ 287
    Onder threshold (ADC < 287) → laser raakt sensor → auto mag rijden → True
    Boven threshold (ADC > 287) → geen contact     → auto mag NIET rijden → False
    """
    try:
        line = ser.readline().decode('utf-8').strip()   # lees één lijn van seriële poort
        sub  = "laser: "                                 # prefix die de microcontroller stuurt
        if sub in line:
            adc_waarde = int(line.split(sub)[1].strip()) # haal de ADC waarde op uit de string
            # Vergelijk met threshold: lage waarde = lasercontact = mag rijden
            laser_contact = adc_waarde < LASER_THRESHOLD_ADC
            print(f"[LASER] ADC={adc_waarde} | threshold={LASER_THRESHOLD_ADC} | contact={'JA' if laser_contact else 'NEE'}")
            return laser_contact                         # True = contact, False = geen contact
    except (ValueError, UnicodeDecodeError):
        pass                                             # ongeldige lijn → negeer
    return False                                         # bij twijfel: geen contact → veilig niet rijden


# ─────────────────────────────────────────────────────────────────────────────
# PAD NAAR COMMANDO'S OMZETTEN
# ─────────────────────────────────────────────────────────────────────────────

def get_turn(current, needed):
    """
    Bepaalt de benodigde draairichting via het 2D-kruisproduct.
    current en needed zijn richtingsvectoren (dr, dc).
    Kruisproduct positief → rechtsaf, negatief → linksaf.
    """
    cross = current[0] * needed[1] - current[1] * needed[0]  # 2D kruisproduct
    dot   = current[0] * needed[0]  + current[1] * needed[1]  # inwendig product

    if dot == 1:                # zelfde richting → geen draai nodig
        return 'forward'
    elif dot == -1:             # tegengestelde richting → 180° keren
        return 'turn_back'
    elif cross > 0:             # kruisproduct positief → rechts draaien
        return 'turn_right'
    else:                       # kruisproduct negatief → links draaien
        return 'turn_left'


def path_to_commands(path):
    """
    Zet een lijst van BFS celposities om naar een lijst van rijcommando's.
    Mogelijke commando's: 'forward', 'turn_left', 'turn_right', 'turn_back'.
    Startrichting is omhoog (-1, 0) want ingang is linksonder en auto rijdt omhoog.
    """
    if not path or len(path) < 2:   # te kort pad → geen commando's
        return []

    current_dir = (0, -1)           # startrichting = omhoog (rij neemt af)
    commands    = []                 # lijst van commando's die gevuld wordt

    for i in range(1, len(path)):                       # loop over opeenvolgende celparen
        dr = path[i][0] - path[i-1][0]                 # rijverschil tussen huidige en vorige cel
        dc = path[i][1] - path[i-1][1]                 # kolomverschil tussen huidige en vorige cel
        needed_dir = (dr, dc)                           # gewenste rijrichting voor deze stap

        if needed_dir != current_dir:                   # richting verandert → draaien nodig
            turn = get_turn(current_dir, needed_dir)    # bereken welke draaibeweging nodig is
            commands.append(turn)                       # voeg draaicommando toe
            current_dir = needed_dir                    # update huidige rijrichting

        commands.append('forward')                      # na eventueel draaien altijd één cel vooruit (dus als niet draaien ook vooruit!)

    return commands                                     # geef volledige lijst van commando's terug


# ─────────────────────────────────────────────────────────────────────────────
# SNELHEIDSREGELING
# ─────────────────────────────────────────────────────────────────────────────

def update_speed(spier_gespannen):
    """
    Past de rijsnelheid geleidelijk aan op basis van de spierspanning.
    Gebruikt de globale variabelen current_speed en last_time.

    Bij spierspanning:  snelheid stijgt geleidelijk (SPEED_RAMP_UP %/s)
    Bij loslaten:       snelheid daalt geleidelijk  (SPEED_RAMP_DOWN %/s)
    Minimum = SPEED_MIN zodra spier gespannen is, maximum = SPEED_MAX.
    Minimum bij ontspannen = 0 (motor kan volledig stoppen).
    """
    global current_speed, last_time         # gebruik globale snelheidsvariabelen

    now = time.time()                       # huidige tijd ophalen
    dt  = now - last_time                   # tijd verlopen sinds laatste update
    last_time = now                         # tijdstip bijwerken voor volgende keer

    if spier_gespannen:
        # Spier gespannen → snelheid geleidelijk verhogen
        current_speed = min(SPEED_MAX, max(SPEED_MIN, current_speed + SPEED_RAMP_UP * dt))
    else:
        # Spier ontspannen → snelheid geleidelijk verlagen tot 0
        current_speed = max(0.0, current_speed - SPEED_RAMP_DOWN * dt)

    return current_speed                    # geef bijgewerkte snelheid terug


# ─────────────────────────────────────────────────────────────────────────────
# SERIËLE POORT INITIALISATIE  (zelfde als spier_enkel_notch)
# ─────────────────────────────────────────────────────────────────────────────

if not TESTMODE and UART_ENABLED: 
    def UART_intit(PORT, BAUD):
        ser = serial.Serial(port=PORT, baudrate=BAUD, timeout=0.1)  # seriële verbinding openen
        if ser.is_open:
            print(f"Connectie geslaagd met {PORT} @ {BAUD} baud")   # verbinding gelukt
        else:
            print(f"Connectie aan poort {PORT} gefaald")  # verbinding mislukt → programma stoppen
            exit()
        return ser

    ser = UART_intit(PORT, BAUD)
    ser_drive = UART_intit(PORT_DRIVE, BAUD) #maakt een tweede seriele connectie voor de microcontroller op de wagen (waarmee we de motoren zullen aansturen)


    ser.write(f's200\n'.encode('ascii'))    # stuur samplesnelheid naar microcontroller (zelfde als spier_enkel_notch)

# ─────────────────────────────────────────────────────────────────────────────
# CAMERA INITIALISATIE  (zelfde als camera_matrix.py en pad_herkenning.py)
# ─────────────────────────────────────────────────────────────────────────────


if TESTMODE:
    img = cv2.imread("data\\render3.png")      #labyrinthe_aruco_perspective.png
    print("testmode enabled!")
else:
    cap = cv2.VideoCapture(1)   # camera starten op index 0 (eerste beschikbare camera)
    
# ─────────────────────────────────────────────────────────────────────────────
# FASE 1: DOOLHOF INLEZEN  (gebaseerd op camera_matrix.py)
# ─────────────────────────────────────────────────────────────────────────────
# Lees het doolhof eenmalig in via de camera vóór de auto begint te rijden.
# De auto staat dan nog stil boven het doolhof (bovenaanzicht camera).

print("Bezig met doolhof inlezen via camera...")

maze_matrix = None

#cv2.imshow("markers", img)
cv2.waitKey()

while True:
    if TESTMODE:
        frame = img
    else:
        frame = cap.read()
    success, warped_image = get_zone(frame, matrix_cols*10, matrix_rows*10)    #leest frame en return een image van de doolhof
    if success:
        cv2.destroyAllWindows()
        break


processed_image = image_threshold(warped_image, 0.35)

#cv2.imshow("warped", warped_image)
cv2.imshow("processed", processed_image)
cv2.waitKey()
maze_matrix = to_matrix(processed_image,12,8)

print("Doolhof matrix succesvol ingelezen.")

# ─────────────────────────────────────────────────────────────────────────────
# FASE 2: BFS PAD BEREKENEN  (gebaseerd op pad_herkenning.py)
# ─────────────────────────────────────────────────────────────────────────────
# Bereken het kortste pad van ingang naar uitgang.
# BFS garandeert het kortste pad → foute/doodlopende wegen worden automatisch vermeden.

print(f"Pad zoeken van {START} naar {END} ...")

print(f"maze matrix: {maze_matrix}")

path = bfs(maze_matrix, START, END)         # BFS uitvoeren op de ingelezen matrix (zelfde als pad_herkenning.py)

input()


if path is None:                            # geen pad gevonden in het doolhof
    print("FOUT: Geen pad gevonden! Controleer de camera of het doolhof.")
    cap.release()
    cv2.destroyAllWindows()
    ser.close()
    exit()

print(f"Pad gevonden: {len(path)} cellen")  # aantal stappen in het pad
print(f"path = {path}")

# Pad tekenen op het frame (gebaseerd op uitgecommentarieerde code in pad_herkenning.py)
if not TESTMODE:
    ret, frame = cap.read()                     # nieuw frame voor visualisatie
    if ret:
        for (x,y) in path:                      # loop over alle cellen in het pad (zelfde als pad_herkenning.py)
            px = int(y * frame.shape[1] / matrix_cols)   # kolom → pixelcoördinaat x
            py = int(x * frame.shape[0] / matrix_rows)   # rij → pixelcoördinaat y
            cv2.circle(frame, (px,py), 1, (0,0,255), -1) # rode stipjes (zelfde als pad_herkenning.py)
        cv2.imshow("Pad visualisatie", frame)   # toon frame met pad ingetekend
        cv2.waitKey(500)                        # even wachten zodat pad zichtbaar is



# Zet het pad om naar rijcommando's
commands = path_to_commands(path)           # lijst van 'forward', 'turn_left', 'turn_right', 'turn_back'
print(f"Commando's ({len(commands)} totaal): {commands}")


# Index om bij te houden welk commando als volgende uitgevoerd wordt
commando_index = 0                          # start bij het eerste commando
current_path_index = 0                      #waar wij zitten in het pad
# ─────────────────────────────────────────────────────────────────────────────
# FASE 3: BASELINE METEN  (zelfde als spier_enkel_notch)
# ─────────────────────────────────────────────────────────────────────────────
# Eerste 5 seconden: auto staat stil, spier ontspannen, baseline wordt gemeten.
# Na 5 seconden wordt de mediaan van de gemeten waarden de baseline.

print("Bezig met filteren. Ctrl+F2 om te stoppen.")     # zelfde print als spier_enkel_notch
print("Houd spier ontspannen voor baseline meting (5 seconden)...")

# ─────────────────────────────────────────────────────────────────────────────
# HOOFDLUS  (structuur gebaseerd op spier_enkel_notch try/while/except)
# ─────────────────────────────────────────────────────────────────────────────

input("Setup klaar, druk een key om te starten")


try:
    while True:                             # oneindige lus (zelfde als spier_enkel_notch)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('o'):
            control_mode = "keyboard"
        if control_mode == "muscle":
            intentie_gas_geven = spier_gespannen
        else:
            # Keyboard mode: Spatiebalk (ASCII 32)
            if key == 32:
                intentie_gas_geven = True
            else:
                intentie_gas_geven = False
        # ── Spiersensor: tellers verhogen (zelfde als spier_enkel_notch) ─────
        baseline_teller += 1               # totale teller voor baselinebepaling

        # ── Seriële lijn lezen (zelfde als spier_enkel_notch) ────────────────
        line = ser.readline().decode('utf-8').strip()   # lees lijn van microcontroller
        sub  = "spierkracht: "                           # prefix voor spierkracht data

        if not line:                        # lege lijn → overslaan
            pass                            #was eerder continue ma we gaan nie alles skippen als de spiersensor nie meewerkt hn

        try:
            # ── Spierkracht waarde uitlezen (zelfde als spier_enkel_notch) ───
            if sub in line:                 # controleer of lijn spierkracht data bevat
                part = line.split(sub)[1]   # deel na de prefix ophalen
                waarde_spierkracht = part.split(",")[0].strip()  # eerste waarde voor komma
                raw_value = float(waarde_spierkracht)            # omzetten naar getal

                # ── Notchfilter toepassen op 50 Hz (zelfde als spier_enkel_notch) ──
                # Enkel notchfilter, GEEN bandpassfilter (zoals in spier_enkel_notch)
                notch_out, zi_notch = signal.lfilter(b_notch, a_notch, [raw_value], zi=zi_notch)
                notch_out = notch_out[0]    # haal scalar uit array van 1 element

                # ── Baseline opbouwen (zelfde als spier_enkel_notch) ─────────
                if baseline_teller < 5*1000/printsnelheid:      # eerste 5 seconden
                    if not type(notch_out) == list:              # zelfde check als spier_enkel_notch
                        baseline_check.append(notch_out)         # voeg toe aan baseline samples
                elif not baseline_done:                          # 5 seconden voorbij, nog geen baseline
                    baseline = median(baseline_check)            # mediaan als baseline (zelfde als spier_enkel_notch)
                    baseline_done = True                         # markeer baseline als gedaan
                    print("\n"*4)
                    print("baseline inputted: ", baseline)       # zelfde print als spier_enkel_notch
                    print("\n"*4)

                # ── Waarden afdrukken en opslaan (zelfde als spier_enkel_notch) ──
#                print(f"Raw: {raw_value:<8} | Filtered: {notch_out:.2f}")  # zelfde format
                lijst_raw.append(raw_value)          # bewaar ruwe waarde voor plot
                lijst_filtered.append(notch_out)     # bewaar gefilterde waarde voor plot

                # ── Spierspanning detecteren per venster (zelfde als spier_enkel_notch) ──
                if baseline_done:
                    teller += 1
                    if teller < check_lines:             # venster nog niet vol
                        if not isinstance(notch_out, list):          # zelfde check als spier_enkel_notch
                            gemiddelde_lijst.append(notch_out - baseline)       # voeg toe aan huidig venster
                    else:                                # venster vol → evalueer
                        ogenbl_gemiddelde = sum(gemiddelde_lijst)/len(gemiddelde_lijst)  # gemiddelde van venster
                        gemiddelde_lijst = []
                        if not baseline is None:         # baseline moet al bepaald zijn (zelfde check)
                            if ogenbl_gemiddelde < 0 - drempel or 0 + drempel < ogenbl_gemiddelde:
                                spier_gespannen = True   # spier is gespannen
                                spier_lijst.append("True")   # historiek bijhouden (zelfde als spier_enkel_notch)
                            else:
                                spier_gespannen = False  # spier is ontspannen
                                spier_lijst.append("False")  # historiek bijhouden
                            print(f"spier gespannen: {spier_gespannen}")
                        teller = 0                       # reset venster teller (zelfde als spier_enkel_notch)

            # ── Laser status lezen ────────────────────────────────────────────
            # De microcontroller stuurt ook "laser: <ADC_waarde>" op dezelfde seriële poort
            laser_ok = lees_laser_status(ser)       # True als laser sensor contact detecteert

            # ── Snelheid bijwerken op basis van spierspanning ─────────────────

            huidige_snelheid = update_speed(intentie_gas_geven)  # geleidelijke ramp-up/down

            """------------
            Camera gedeelte
            -------------"""

            # ── Oriëntatie bepalen via camera (gebaseerd op orientatie.py) ────
            ret, frame = cap.read()                 # lees frame van camera
            if ret:
                hoek, frame2 = bepaal_orientatie(frame)  # hoek + geannoteerd frame
                cv2.imshow("Origineel", frame2)      # toon frame met oriëntatiepijl (zelfde als orientatie.py)
                cv2.imshow("Threshold", cv2.threshold(                # toon threshold (zelfde als orientatie.py)
                    cv2.GaussianBlur(cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY), (5,5), 0),
                    200, 255, cv2.THRESH_BINARY)[1])

            # ── Navigatie uitvoeren (enkel als laser ok EN baseline bepaald) ──
            if commando_index < len(commands) and baseline_done:

                if not laser_ok:
                    # Laser detecteert GEEN contact → auto mag NIET rijden → motoren stoppen
                    stop_motors()
                    print("[LASER] Geen contact gedetecteerd → motoren geblokkeerd!")

                else:       # Laser detecteert contact → auto mag rijden
                    command = process_frame(frame, path, current_path_index)
                    match command:
                        case "NO_MARKER":
                            print("[info] no marker found!!\ncontinuing with current task")
                        case "NEXT_INSTRUCTION":
                            commando_index +=1
                            current_path_index +=1 #ook al moet hij draaien gaat hij na het draaien altijd moeten voortrijden en dus naar het volgende vak van het doolhof rijden
                        case "FINISHED": 
                            commando_index = len(commands) + 1
                        case "KEEP_GOING":
                            pass
                    
                    cmd = commands[commando_index]  # huidig te uit te voeren commando

                    if cmd == 'forward':
                        # Rijcommando: rijd aan huidige snelheid als spier gespannen is
                        if huidige_snelheid > 0:
                            move_forward(huidige_snelheid)   # rijd vooruit
                        else:
                            stop_motors()                    # snelheid = 0 → stop

                    elif cmd in ('turn_left', 'turn_right', 'turn_back'):       #als hij moet draaien doe hij het in een keer (script stopt tot dat hij gedraaid is) dit betekent dat we direct naar de volgende commando gaan 
                        # Draaicommando: wacht tot snelheid laag genoeg is
                        if huidige_snelheid > MAX_SPEED_TO_TURN:
                            # Nog te snel → rem af (motoren stoppen, ramp-down loopt door)
                            stop_motors()
                            print(f"[NAV] Afremmen voor bocht ({cmd}) | snelheid={huidige_snelheid:.1f}%")
                        else:
                            # Veilige snelheid bereikt → draaibeweging uitvoeren
                            draai_snelheid = max(huidige_snelheid, SPEED_MIN)  # minstens SPEED_MIN

                            if cmd == 'turn_left':
                                turn_left_90(draai_snelheid)         # draai 90° links

                            elif cmd == 'turn_right':
                                turn_right_90(draai_snelheid)        # draai 90° rechts

                            elif cmd == 'turn_back':
                                # 180° keren = twee keer 90° links draaien
                                turn_left_90(draai_snelheid)         # eerste 90°
                                time.sleep(0.2)                      # kort wachten tussen de twee draaibewegingen
                                turn_left_90(draai_snelheid)         # tweede 90°

                            commando_index += 1                      # ga naar volgend commando

            elif commando_index >= len(commands) and baseline_done:
                # Alle commando's uitgevoerd → doolhof voltooid
                stop_motors()                                        # motoren stoppen
                print("[NAV] Doolhof succesvol voltooid!")
                break                                                # verlaat de hoofdlus

            # ── Status afdrukken ──────────────────────────────────────────────
            print(f"[STATUS] spier={'AAN' if spier_gespannen else 'UIT'} | "
                  f"snelheid={huidige_snelheid:.1f}% | "
                  f"laser={'OK' if laser_ok else 'GEBLOKKEERD'} | "
                  f"stap {commando_index}/{len(commands)}")

        except ValueError:
            print("value error!!")
            continue                         # ongeldige waarde op seriële poort → overslaan (zelfde als spier_enkel_notch)

        # ── Toets 'q' om te stoppen (zelfde als camera_matrix / orientatie) ──
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Gestopt door gebruiker.")
            break
    time.sleep(0.01)

# ─────────────────────────────────────────────────────────────────────────────
# AFSLUIT BLOK  (exact zelfde structuur als spier_enkel_notch)
# ─────────────────────────────────────────────────────────────────────────────

except KeyboardInterrupt:
    # Ctrl+C ingedrukt
    None

finally:
    # Altijd uitvoeren bij afsluiten (zelfde als spier_enkel_notch)
    stop_motors()                               # zeker zijn dat motoren gestopt zijn
    cap.release()                               # camera vrijgeven (zelfde als camera_matrix.py)
    cv2.destroyAllWindows()                     # alle vensters sluiten (zelfde als camera_matrix.py)
    if ser.is_open:                             # zelfde check als spier_enkel_notch
        ser.close()
        print("Poort gesloten.")                # zelfde print als spier_enkel_notch
