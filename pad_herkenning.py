# -*- coding: utf-8 -*-
"""
Created on Wed Mar 11 09:05:37 2026

@author: zahid
"""
#grond is zwart dus we gaan zwarte tape moeten gebruiken
import cv2
from collections import deque
import matplotlib.pyplot as plt
import numpy as np
import queue

# BFS pathfinding (= breadth first search)
def bfs(maze, start, end):
    rows, cols = len(maze), len(maze[0])
    queue = deque()
    queue.append((start, [start]))
    visited = set()
    visited.add(start)
    directions = [(1,0),(-1,0),(0,1),(0,-1)]  # onder, boven, rechts, links

    while queue:
        (x,y), path = queue.popleft()
        if (x,y) == end:
            return path
        for dx,dy in directions:
            nx, ny = x+dx, y+dy
            if 0<=nx<rows and 0<=ny<cols:
                if maze[nx][ny]==0 and (nx,ny) not in visited:
                    visited.add((nx,ny))
                    queue.append(((nx,ny), path + [(nx,ny)]))
    return None

def BFS(maze):
    start_state = {'pos':[1, 1],'parent':None}
    q = queue.Queue()
    q.put(start_state)
    
    while q.empty() == False:
        state = q.get()
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        for zet in directions:
            new_pos = (state['pos'][0]+zet[0],state['pos'][1]+zet[1])
            if maze[new_pos[0],new_pos[1]] == 0:
                new_state = {'pos':new_pos,'parent':state}
                if new_pos == (len(maze)-2,len(maze)-1) :
                    print("Oplossing gevonden!!!")
                    return new_state
                else: 
                    q.put(new_state)
    return None

def draw_maze(maze, visited=None,path=None):
    fig, ax = plt.subplots(figsize=(10,10))
    
    # Set the border color to white
    fig.patch.set_edgecolor('white')
    fig.patch.set_linewidth(0)

    ax.imshow(maze, cmap=plt.cm.binary, interpolation='nearest')
    
    # Draw the solution path if it exists
    if visited is not None:
        x_coords = [x[1] for x in visited]
        y_coords = [y[0] for y in visited]
        ax.scatter(x_coords, y_coords, color='red', linewidth=2)
    if path is not None:
        x_coords = [x[1] for x in path]
        y_coords = [y[0] for y in path]
        ax.plot(x_coords, y_coords, color='blue', linewidth=2)
    
    ax.set_xticks([])
    ax.set_yticks([])
    
    # Draw entry and exit arrows
    ax.arrow(0, 1, .4, 0, fc='green', ec='green', head_width=0.3, head_length=0.3)
    ax.arrow(maze.shape[1] - 1, maze.shape[0]  - 2, 0.4, 0, fc='blue', ec='blue', head_width=0.3, head_length=0.3)
    
    plt.show()


matrix = [[0,0,0,1,0,1,1,1]
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
,[1,0,0,0,0,0,1,0]]


    
if False:
    # bestaande code gebruiken van de camera 
    cap = cv2.VideoCapture(0)
    matrix_rows = 500
    matrix_cols = 500

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)
        _, thresh = cv2.threshold(blur, 100, 255, cv2.THRESH_BINARY_INV)

        # bestaande maze_matrix
        small_thresh = cv2.resize(thresh, (matrix_cols, matrix_rows), interpolation=cv2.INTER_NEAREST)
        maze_matrix = small_thresh // 255

        # Start en einde voor BFS
        start = (0,0)                   # linksboven
        end = (matrix_rows-1, matrix_cols-1)  # rechtsonder
        
        
        """path = bfs(maze_matrix, start, end)  # kortste pad vinden

        # Pad tekenen op orig.frame
        if path:
            for (x,y) in path:
                px = int(y * frame.shape[1] / matrix_cols)
                py = int(x * frame.shape[0] / matrix_rows)
                cv2.circle(frame, (px,py), 1, (0,0,255), -1)  # rode stipjes""" 

        # show the beelden
        cv2.imshow("Origineel", frame)
        cv2.imshow("Threshold", thresh)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()

    cv2.destroyAllWindows()

