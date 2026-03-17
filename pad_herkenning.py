# -*- coding: utf-8 -*-
"""
Created on Wed Mar 11 09:05:37 2026

@author: zahid
"""
import cv2     # OpenCV → camera en beeldverwerking
from collections import deque  # deque → snelle wachtrij voor BFS algoritme
import matplotlib.pyplot as plt #matplotlib → visualisatie van doolhof en pad
import numpy as np              #numpy → werken met matrices
import queue                    #queue → tweede type wachtrij

# BFS pathfinding (= breadth first search)
def bfs(maze, start, end):                   # functie die het kortste pad zoekt in het doolhof
    rows, cols = len(maze), len(maze[0])     # aantal rijen en kolommen van het doolhof bepalen
    queue = deque()                          # lege wachtrij maken voor BFS
    queue.append((start, [start]))           # startpositie + huidig pad toevoegen aan de wachtrij
    visited = set()                          # set maken voor bezochte cellen
    visited.add(start)                       # startcel markeren als bezocht
    directions = [(1,0),(-1,0),(0,1),(0,-1)] # mogelijke bewegingen: onder, boven, rechts, links

    while queue:                             # blijf zoeken zolang er cellen in de wachtrij zitten
        (x,y), path = queue.popleft()        # haal de eerste cel uit de wachtrij
        if (x,y) == end:                     # controleer of eindpunt bereikt is
            return path                      # geef het gevonden pad terug
        for dx,dy in directions:             # kijk naar alle buurcellen
            nx, ny = x+dx, y+dy              # bereken positie van buurcel
            if 0<=nx<rows and 0<=ny<cols:    # controleer of buurcel binnen de matrix ligt
                if maze[nx][ny]==0 and (nx,ny) not in visited:  #controleer of het pad is en nog niet bezocht
                    visited.add((nx,ny))     # markeer cel als bezocht
                    queue.append(((nx,ny), path + [(nx,ny)]))  # voeg nieuwe cel + pad toe aan wachtrij
    return None                              # als geen pad gevonden wordt

def BFS(maze):          #alternatieve BFS implementatie
    start_state = {'pos':[1, 1],'parent':None}  #startpositie en parent state opslaan
    q = queue.Queue()            # nieuwe wachtrij maken
    q.put(start_state)           # startstaat toevoegen aan wachtrij
    
   while q.empty() == False:                # zolang de wachtrij niet leeg is
        state = q.get()                      # haal eerste staat uit wachtrij
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)] # mogelijke bewegingen
        for zet in directions:               # loop door alle mogelijke bewegingen
            new_pos = (state['pos'][0]+zet[0],state['pos'][1]+zet[1]) # nieuwe positie berekenen
            if maze[new_pos[0],new_pos[1]] == 0: # controleren of positie pad is
                new_state = {'pos':new_pos,'parent':state} # nieuwe staat maken
                if new_pos == (len(maze)-2,len(maze)-1) :  # controle of eindpunt bereikt is
                    print("Oplossing gevonden!!!")         # melding dat pad gevonden is
                    return new_state                       # oplossing teruggeven
                else: 
                    q.put(new_state)                       # anders nieuwe staat toevoegen aan wachtrij
    return None                                            # geen oplossing gevonden

def draw_maze(maze, visited=None,path=None):   # functie om doolhof en pad te visualiseren
    fig, ax = plt.subplots(figsize=(10,10))    # figuur maken voor visualisatie
    
    fig.patch.set_edgecolor('white')           # randkleur van figuur instellen
    fig.patch.set_linewidth(0)                 # randdikte verwijderen

    ax.imshow(maze, cmap=plt.cm.binary, interpolation='nearest') # doolhof tekenen (zwart/wit)
    
    if visited is not None:                    # als bezochte cellen bestaan
        x_coords = [x[1] for x in visited]     # x-coördinaten verzamelen
        y_coords = [y[0] for y in visited]     # y-coördinaten verzamelen
        ax.scatter(x_coords, y_coords, color='red', linewidth=2) # bezochte cellen rood tekenen

    if path is not None:                       # als oplossing pad bestaat
        x_coords = [x[1] for x in path]        # x-coördinaten pad
        y_coords = [y[0] for y in path]        # y-coördinaten pad
        ax.plot(x_coords, y_coords, color='blue', linewidth=2) # oplossing pad blauw tekenen
    
    ax.set_xticks([])                          # x-as ticks verwijderen
    ax.set_yticks([])                          # y-as ticks verwijderen
    
    ax.arrow(0, 1, .4, 0, fc='green', ec='green', head_width=0.3, head_length=0.3) # startpijl tekenen
    
    # eindpijl
    ax.arrow(maze.shape[1] - 1, maze.shape[0]  - 2, 0.4, 0, fc='blue', ec='blue', head_width=0.3, head_length=0.3) 
    plt.show()                                 # figuur tonen


'''matrix = [[0,0,0,1,0,1,1,1]
,[0,1,0,1,0,1,0,0]
,[0,1,0,0,0,1,0,1]
,[0,1,1,1,0,1,0,0]
,[0,0,0,1,0,1,1,0]
,[0,1,0,1,0,0,0,0]
,[0,1,1,1,1,1,0,1]
,[0,0,0,0,1,1,1,1]
,[1,0,1,0,0,0,0,1]
,[0,0,1,1,1,1,0,1]
,[1,1,1,0,1,0,0,0]
,[1,0,0,0,0,0,1,0]]'''


    
if False:                                       # deze code wordt NIET uitgevoerd (handig om tijdelijk uit te zetten)
    # bestaande code gebruiken van de camera 
    cap = cv2.VideoCapture(0)                       # start de camera (index 0)
    matrix_rows = 500                               # aantal rijen van de matrix (resolutie)
    matrix_cols = 500                               # aantal kolommen van de matrix

    while True:                                     # oneindige lus → blijft frames lezen
        ret, frame = cap.read()                     # lees een frame van de camera
        if not ret:                                 # als het lezen mislukt
            break                                   # stop de lus

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)   # zet beeld om naar grijswaarden
        blur = cv2.GaussianBlur(gray, (5,5), 0)          # blur om ruis te verminderen
        _, thresh = cv2.threshold(blur, 100, 255, cv2.THRESH_BINARY_INV)  # zwart-wit beeld maken

        # bestaande maze_matrix
        small_thresh = cv2.resize(thresh, (matrix_cols, matrix_rows), interpolation=cv2.INTER_NEAREST)  # schaal beeld naar matrix grootte
        maze_matrix = small_thresh // 255               # zet pixelwaarden om naar 0 (pad) en 1 (muur)

        # Start en einde voor BFS
        start = (0,0)                   # startpunt linksboven
        end = (matrix_rows-1, matrix_cols-1)  # eindpunt rechtsonder

        # show the beelden
        cv2.imshow("Origineel", frame)        # toon originele camerabeeld
        cv2.imshow("Threshold", thresh)       # toon zwart-wit (threshold) beeld

        if cv2.waitKey(1) & 0xFF == ord('q'): # als je op 'q' drukt
            break                             # stop het programma

    cap.release()                             # sluit de camera
    cv2.destroyAllWindows()                   # sluit alle OpenCV vensters
