import math
from random import random
import time
import numpy as np

#prelimeraries
#1. initialization
neighbours = []
primitiveMap = []
gridCells = []
width = 12
height = 12
start = [(0,1,0),0.1]
goal = [(11,5,0),0.1] #[(3,2,0),0.1] # [9,11,-1,0] [4,8,-1,0]
obstaclesList = [[(0,11,1),0],[(0,0,1),0],[(2,3,1),0],[(5,11,1),0],[(6,11,1),0],
                 [(10,5,1),0], [(7,2,1),0],[(3,8,1),0], [(7,8,-1),0],[(7,10,-1),0], 
                 [(9,8,-1),0],[(10,9,-1),0],[(9,10,1),0]]
#2. Map creation
for i in range(width): #ligne, x
    for j in range(height): #colonnes, y
        gridCells.append([i,j,0])
#utilisation d'une comprehension de generateur
primitiveMap = list(list([tuple(item),0.1]) for item in gridCells)

#transformer la liste en dictionnaire avec comme clé : Coord et val: celle des pheronomes
#initialisation des pheronomes a 0.1
pheromone_list = []
for i in range(width):
    for j in range(height):
        pheromone_list.append(0.1)
#definitions des fonctions
def addObstacles(obstaclesList, gridCells):
    for i in range(len(gridCells)):
        for j in range(len(obstaclesList)):
            if obstaclesList[j][0][:2] == gridCells[i][0][:2]:
                gridCells[i][0] = obstaclesList[j][0]
    return gridCells
    
def neighboursFunc(position,gridCells):
    #definition of four movements:
    neighbours = []
    movements = [(0,-1,0), (0, 1,0), (1,0,0), (-1, 0,0)]
    for i in movements:
        neighbour = (position[0] + i[0], position[1] + i[1], 0) #a retravailler
        #print(neighbour)
        x,y,_ = neighbour
        if 0<=x<width and 0<=y<height and neighbour in gridCells: #verifier voir si ce n'est pas obstacle
            neighbours.append(neighbour)
    #print(neighbours) #__ example [(0, 0), (0, 2), (1, 1)] for tuple (0,1,0)
    return neighbours
    
def heuristic(position, goal):
    xCurrent, yCurrent,_ = position
    xG, yG, _ = goal
    return math.sqrt((xG-xCurrent)**2 + (yG-yCurrent)**2)
    
def updatePheromone(path,gridCells, pheromone, phEvap):
    global flag
    if flag == True:
        pheromone = pheromone/5
    
    for i in range(len(gridCells)):
        #pheronome evaporation
        #if gridCells[i][1] > 0:
            #gridCells[i][1] -= phEvap
            #pheromone update
        for j in range(len(path)):
            if path[j] == gridCells[i][0][:3]:
                gridCells[i][1] += pheromone * (1 - phEvap)
    return gridCells

def findBestPath(allPath): #based on pheromones
    validePath = [path for path in allPath if goal[0] in path]
    lenOfPaths = [len(var) for var in validePath]
    if lenOfPaths != []:
        bestPath_idx = lenOfPaths.index(min(lenOfPaths))
        bestPath = validePath[bestPath_idx]
    else:
        print("error - findBestPath, lenOfPaths is empty")  
        bestPath = []
    return bestPath
def nextPose(position, pheromones, pheromone_list,prev_pose):
    possibles_moves = []
    pheromone_levels = []
    distance2Goal = []
    denominatorValue = []
    global flag
    i = 0
    possibles_moves = neighboursFunc(position, positions)
    nbr_moves = len(possibles_moves)
    #print(nbr_moves)
    #idee : reduire la liste des voisins en fonction de ce qui est deja dans la closedList
    # Pré-filtrage pour éliminer les mouvements déjà visités
    filtered_moves = [move for move in possibles_moves if move not in prev_pose]
    
    if not filtered_moves:  # Si aucun mouvement n'est valide après le filtrage
        print("Aucun mouvement valide restant, flag mis à True")
        pose = []
        return pose
    else:
        for move in filtered_moves:
            x, y, _ = move
            idx = x * height + y
            pheromone_levels.append(pheromone_list[idx])
            distance2Goal.append(heuristic(move, goal[0]))

    #total_phero = sum(pheromone_levels)
    for i in range(len(pheromone_levels)):
        denominatorValue.append((pheromone_levels[i]**alpha)*(distance2Goal[i]**beta))
    denominator = sum(denominatorValue) + 1e-10
    
    probabilities = [((level**alpha)*(dis**beta))/denominator for level, dis in zip(pheromone_levels,distance2Goal)]
    #print(probabilities)
    condition = True
    while condition:
        next_pose_idx = np.random.choice(range(len(possibles_moves)))#,p=probabilities)
        #print(next_pose_idx)
        x_next,y_next,_ = possibles_moves[next_pose_idx]
        pose = possibles_moves[next_pose_idx]
        if pose not in prev_pose: #deja traité plus haut, donc, revoir cette fonction ...
            condition = False
            x,y,_ = position
            #print(pose)
    #print(flag)
    return pose

#DEBUT ACO
#ACO parameters initialization
alpha = 4 #pheromone importance
beta = 3 #distance importance
phEvap = 0.9
pheromone = 10
primitiveMap = addObstacles(obstaclesList,primitiveMap)
n_iterations = 20
nbAnt = 15
#global flag 
flag =  False
#ant creation
antsList = list()
antsTrack = []
positions = [pose[0] for pose in primitiveMap]
for i in range(nbAnt):
    antsList.append([(i+1,0,0),list()]) #l'incrementation c'est juste pour la verif.

for a in range(n_iterations):
    print("")
    print("iteration n° ", a,"_____________________")
    for i in range(nbAnt):
        current_p = antsList[i][0]
        antsList[i][0] = start[0]
        antsList[i][1].append(antsList[i][0])
        #print("ant n", i, "________________________")
        flag = False
        #print(antsList)
        while antsList[i][0] != goal[0] and not flag:
            #print(flag)
            pose = nextPose(antsList[i][0],pheromone,pheromone_list,antsList[i][1])
            if pose != []:
                antsList[i][0] = pose
                antsList[i][1].append(antsList[i][0])
                #print("---------------------")
            else :
                flag = True
                #print("flag is set to True now")
            if antsList[i][0] == goal[0] or flag:
                #print("Goal reached or no valid moves left, updating pheromones.")
                primitiveMap = updatePheromone(antsList[i][1], primitiveMap, pheromone, phEvap)
                #print("Updated Map with Pheromones:")
                antsTrack.append(antsList[i][1]) #enregistrer la bonne trajectoire
                antsList[i][1] = []
    #print(antsTrack)
    bestPath = findBestPath(antsTrack)
    print("best path for iteration",a,"=",bestPath)