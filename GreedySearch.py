import math
from typing import Optional
import time

neighbours = []
gridCells = []
shortestPath = []
closedList = []
openList = []
h = []
width = 12
height = 12
start = [0,1,0,0]
goal = [11,5,-1,0] # [9,11,-1,0] [4,8,-1,0]
obstaclesList = [[0,11,-1,0],
                 [0,0,-1,0],
                 [2,3,-1,0],
                 [5,11,-1,0],
                 [6,11,-1,0],
                 [10,5,-1,0],
                 [7,2,-1,0],
                 [3,8,-1,0], [7,8,-1,0], [7,10,-1,0], [9,8,-1,0],[10,9,-1,0],
                 [9,10,-1,0]]

#map creation
for i in range(width):
    for j in range(height):
        gridCells.append([i,j,-1,0])
        
def heuristic(goal,position):
    xG, yG, _, _ = goal
    x, y, _, _ = position
    hVal = math.sqrt((x-xG)**2 + (y-yG)**2)
    return hVal

def neighboursFunction(position,gridCells):
    #definition of four movements:
    movements = [[0,-1],
                 [0, 1],
                 [1, 0],
                 [-1, 0]]
    for i in movements:
        neighbour = [position[0] + i[0], position[1] + i[1],-1,0]
        if neighbour in gridCells and gridCells[gridCells.index(neighbour)][3] == 0:
            neighbours.append(neighbour)
        #print(neighbours)
    return neighbours
    
def addObstacles(obstaclesList, gridCells):
    for obstacle in obstaclesList:
        for grid in gridCells:
            if obstacle == grid:
                grid[3] = 1
    return gridCells
                
def petiteValeur(hList):
    interVal = hList[0]
    idx = 1
    for idx in range(len(hList)):
        if hList[idx] <= interVal:
            interVal = hList[idx] 
    return interVal
    
def idxPetiteVal(hList, petiteVal):
    for i in range(len(hList)):
        if petiteVal == hList[i]:
            break
    return i

def appendOpenList(neighbours, openList):
    for neighbour in neighbours:
        if neighbour not in openList:
            openList.append(neighbour)
    return openList
    
def removeFrom(idx, openList, neighbours):
    n_idx = -len(neighbours)+idx #logique negative
    print(n_idx)
    openList.pop(n_idx)
    n_idx = 0
    return openList

def is_not_in_list(data, myList):
    return data not in myList            
def aSearchAlgo(goal, start, gridCells,closedList,openList,neighbours,h):
    #initialization
    g=1
    gridCells = addObstacles(obstaclesList,gridCells)
    #print(gridCells)
    currentNode = start
    closedList.append(start)
    openList.append(start)
    #loop
    while openList:
        neighbours = neighboursFunction(currentNode,gridCells)
        for neighbour in neighbours:
            if neighbour[2]== -1 and neighbour[2]!= 0 and neighbour[2]<g:
                neighbour[2]=g
                #print(neighbour)
            hVal = heuristic(goal,neighbour)
            h.append(hVal)
            if neighbour not in openList :
                openList.append(neighbour)
        #verif plus petite val de h
        lowestH = petiteValeur(h)
        bestH_idx = idxPetiteVal(h,lowestH)
        
        #ajout a la liste fermée et retrait de la liste ouverte
        if neighbours[bestH_idx] not in closedList:
            closedList.append(neighbours[bestH_idx])
            openList.remove(neighbours[bestH_idx])
        if currentNode in openList:
            openList.remove(currentNode)
        currentNode = neighbours[bestH_idx]
        print(g)
        print("")
        #condition d'arrêt
        if currentNode[:2] == goal[:2] :
            print("shortest path found ...")
            path = closedList
            #print(path)
            return path
        else:
            h = []
            neighbours=[]
        print(closedList)
        g+=1
        #time.sleep(2)
