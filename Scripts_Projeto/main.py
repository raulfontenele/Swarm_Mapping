import sim
import time
import math
from Robot import Robot
from Lidar import Lidar
from Node import Node
from Map import Map
from AuxiliarFunctions import AuxiliarFunctions
from Save import saveCoord,saveEdge,saveMap
from debug import logNodeInfo
import json
from PathPlanning import PathPlanning
import datetime

def Exploring(robotFile):
    #Definição de variáveis
    motorsObject = []
    velocity = 0.4
    radius = 0.25
    #extRadius = radius/math.cos(math.pi/6)
    global mapping
    tInit = datetime.datetime.now()
    #Pegar informações do json do robô
    fileJson = open(robotFile)
    robotInfo = json.load(fileJson)
    
    #Pegar o objeto do robô
    returnCode,khepera=sim.simxGetObjectHandle(clientID,robotInfo["robotBody"],sim.simx_opmode_blocking)

   #Criar objetos do acelerômetro, Lidar e giroscópio
    lidar = Lidar(robotInfo["lidar"],clientID,28,2)
     
    #Pegar objeto dos motores manipuláveis
    for i in range(2):
        returnCode,handle=sim.simxGetObjectHandle(clientID,robotInfo["motorsName"][i],sim.simx_opmode_blocking)
        motorsObject.append(handle)
        
    #Criar o objeto do robô
    robot = Robot(clientID,robotInfo["id"],motorsObject,khepera,lidar)
    
    #Tempo para estabelecer conexões com o servidor
    time.sleep(0.1)
    
    initPose = robot.getAbsolutePosition(False)
    mapping.initGoalsNode(robotInfo["id"], initPose)
    neighborhood,angles = robot.scanAround(velocity*0.8,mapping)
    
    
    mapping.addMapPoint(initPose,neighborhood,angles)
    mapping.addVisitedNode(initPose)
    mapping.addNoneVisitedNode(neighborhood)
    
    while len(mapping.noneVisitedList) >0 :
        '''
            verificar se existe algum visinho disponível para ser visitado, ou seja, não estar na lista de nós visitados
            caso não, planejar o caminho para um nó não visitado
        '''
        #if neighborhood != None:
        
        try:
            neighborCoord,neighborAngle = mapping.checkAvailability(neighborhood,angles)
        except Exception:
            neighborCoord = None
        
            
        if neighborCoord != None:
            #Visitar o nó 
            print("Nó não visitado")
            print("Coordenadas destino: " + str(neighborCoord) )
            #Definir o próximo objetivo/ objetivo final e visita-lo
            mapping.updateGoals(neighborCoord, neighborCoord, robotInfo["id"])
            robot.rotateTo(neighborAngle, velocity*0.8)
            distParent = robot.moveFoward(2*radius, 0, velocity*3)
            print("Distancia deslocada: " + str(distParent) )
            
            '''
                Uma vez que se chegou no nó não visitado, deve-se escanear em volta, e adicionar os visinhos, não visitados,
                a lista de nós não visitados, atualizar o nó atual do mapa e dizer que o nó atual é filho do nó anterior.
                Também deve-se adiconar o nó atual a lista de visitados, assim como dizer que o atual nó, visinho do pai, foi visitado
            '''
            
            currentPose = robot.getAbsolutePosition(False)
            mapping.visitedNode(currentPose)
            
            neighborhood,angles = robot.scanAround(velocity*0.8,mapping)
            mapping.addNoneVisitedNode(neighborhood)
            
            mapping.addMapPoint(currentPose,neighborhood,angles)
            
        else:
            if len(mapping.noneVisitedList) > 0:
                #Escolher um nó para ir e planejar o caminho
                node_start = Node(currentPose)
                node_start.gcost = 0
                nextGoalIndex = 0
                
                # Escolher o ponto futuro apenas se o mesmo não for objetivo de alguém
                while True:
                    if mapping.checkGoalAnother(mapping.noneVisitedList[0]) == True:
                        nextGoalIndex +=1
                    else:
                        node_goal = Node(mapping.noneVisitedList[nextGoalIndex])
                        break
                
                mapping.updateGoals(currentPose, mapping.noneVisitedList[nextGoalIndex], robotInfo["id"])
                    
                #node_goal = Node(mapping.noneVisitedList[0])
                #goal = mapping.noneVisitedList[0]
                print("planejar o caminho")
                path = PathPlanning(mapping,node_start,node_goal)
                route = path.AStarAlgorithm()
                print("Posição atual:" + str(robot.getAbsolutePosition(False)))
                print("Rota")
                print(route)
                #Ir até o lugar planejado
                for index in reversed(range(1,len(route))):
                    current_position = robot.getAbsolutePosition(False)
                    #angle = AuxiliarFunctions.oppositeAngle(route[index][1][2])
                    distance,angle = AuxiliarFunctions.CalcAngleDistance(current_position,route[index-1])
                    print("Distância:" + str(distance) + "// Angulo:" + str(angle))
                    robot.rotateTo(angle, velocity*0.8)
                    robot.moveFoward(distance, 0, velocity*3)
                    print("Posição atual e objetivo:")
                    print(robot.getAbsolutePosition(False))
                    print(route[index-1])
                

                print("Terminou com posição atual e objetivo:")
                print(robot.getAbsolutePosition(False))
                print(route[0])
                
                currentPose = robot.getAbsolutePosition(False)
                mapping.visitedNode(currentPose)
                
                neighborhood,angles = robot.scanAround(velocity*0.8,mapping)
                mapping.addNoneVisitedNode(neighborhood)
                
                mapping.addMapPoint(currentPose,neighborhood,angles)
                
    #saveCoord(mapping.visitedList)
    saveMap(mapping.structMap, "map")
    tFinal = datetime.datetime.now()
    deltaT = tFinal- tInit
    print(deltaT)
    #saveEdge(mapping.edgeMap,'arestas')
    print("Terminou")

print ('Program started')

#Variáveis do projeto
#motorsName = ['Pioneer_p3dx_leftMotor','Pioneer_p3dx_rightMotor']
#robotName = 'Pioneer_p3dx'
motorsName = ['K3_leftWheelMotor','K3_rightWheelMotor']
robotName = 'K3_robot'
robotBody = 'K3_bodyPart1'
motorsObject = []
wallReference = 'ConcretBlock'

velocity = 0.5
radius = 0.25
extRadius = radius/math.cos(math.pi/6)

contagem = 0

sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim

mapping = Map(extRadius)
Exploring('robot1.json')



# Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
sim.simxGetPingTime(clientID)

# Now close the connection to CoppeliaSim:
sim.simxFinish(clientID)



