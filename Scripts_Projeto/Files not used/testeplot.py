import sim
import time
import math
from Robot import Robot
from Lidar import Lidar
from Node import Node
from Map import Map
from AuxiliarFunctions import AuxiliarFunctions
from Save import saveCoord,saveEdge,saveMap,saveDebug
from debug import logNodeInfo
import json
from PathPlanning import PathPlanning
import datetime
import threading



def getMap():
    file = open('map.txt','r')
    lines = file.readlines()
    
    mapa = []
    
    for line in lines:
        line = line.replace("'",'"')
        struct =json.loads(line)
        
        structWrite = {
            "nodeCoord":    struct["nodeCoord"],
            "neighborhood": struct["neighborhood"],
            "angles":       struct["angles"],
            "robotId":      struct["robotId"]
        }
        mapa.append(structWrite)
    return mapa

radius = 0.5
extRadius = radius/math.cos(math.pi/6)

contagem = 0

mapping = Map(extRadius)

mapa = getMap()
mapping.structMap = mapa

node_start = Node([1.9702228307724, 4.313887119293213, 0.035470686852931976])
node_start.gcost = 0
node_goal = Node([1.0138203501701355, -0.8659255465630812, 0.035322099924087524])


path = PathPlanning(mapping,node_start,node_goal)
route = path.AStarAlgorithm()
