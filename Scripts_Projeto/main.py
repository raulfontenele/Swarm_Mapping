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
import threading

def Exploring(robotFile,comPort):
    #Definição de variáveis
    motorsObject = []
    velocity = 0.9
    #velocity = 0.4
    radius = 0.25
    #extRadius = radius/math.cos(math.pi/6)
    global mapping
    tInit = datetime.datetime.now()
    #Pegar informações do json do robô
    fileJson = open("robot" + str(robotFile) + ".json")
    robotInfo = json.load(fileJson)
    
    #Testar sem fechar conexões existentes
    #sim.simxFinish(-1) # just in case, close all opened connections
    clientID=sim.simxStart('127.0.0.1',comPort,True,True,5000,5) # Connect to CoppeliaSim
    
    #Pegar o objeto do robô
    returnCode,khepera=sim.simxGetObjectHandle(clientID,robotInfo["robotBody"],sim.simx_opmode_blocking)

   #Criar objetos do acelerômetro, Lidar e giroscópio
    lidar = Lidar(robotInfo["lidar"],clientID,robotInfo["id"])
     
    #Pegar objeto dos motores manipuláveis
    for i in range(2):
        returnCode,handle=sim.simxGetObjectHandle(clientID,robotInfo["motorsName"][i],sim.simx_opmode_blocking)
        motorsObject.append(handle)
        
    #Criar o objeto do robô
    robot = Robot(clientID,robotInfo["id"],motorsObject,khepera,lidar)
    
    #Tempo para estabelecer conexões com o servidor
    time.sleep(0.2)
    
    mapping.initStatusMap(robotInfo["id"], "stopping")
    
    initPose = robot.getAbsolutePosition(False)
    print("Posição Inicial do robô " + str(robotInfo["id"]) + ": " + str(initPose))
    mapping.initGoalsNode(robotInfo["id"], initPose)
    mapping.initStatusMap(robotInfo["id"], "exploring")
    neighborhood,angles = robot.scanAround(velocity*0.8,mapping)
    
    
    mapping.addMapPoint(initPose,neighborhood,angles)
    mapping.addVisitedNode(initPose)
    mapping.addNoneVisitedNode(neighborhood)
    mapping.updateVisitedNode()
    
    '''
        Não pode haver nós a serem visitados e nem os visinhos podem estar explorando ou se mexendo
    '''
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
            print("Nó não visitado do robô:" + str(robotInfo["id"]))
            
            #Definir o próximo objetivo/ objetivo final e visita-lo
            mapping.updateGoals(neighborCoord, neighborCoord, robotInfo["id"])
            mapping.updateStatus(robotInfo["id"], "moving")
            
            robot.rotateTo(neighborAngle, velocity*0.8)
            distParent = robot.moveFoward(2*radius, 0, velocity*3)
            print("Distancia deslocada: " + str(distParent) + " pelo robô "+ str(robotInfo["id"]))
            
            
            '''
                Uma vez que se chegou no nó não visitado, deve-se escanear em volta, e adicionar os visinhos, não visitados,
                a lista de nós não visitados, atualizar o nó atual do mapa e dizer que o nó atual é filho do nó anterior.
                Também deve-se adiconar o nó atual a lista de visitados, assim como dizer que o atual nó, visinho do pai, foi visitado
            '''
            
            currentPose = robot.getAbsolutePosition(False)
            
            print("Coordenadas destino: " + str(neighborCoord) )
            print("Coordenada final:" + str(currentPose) + " do robô " + str(robotInfo["id"]))
            
            mapping.visitedNode(currentPose)
            
            mapping.updateStatus(robotInfo["id"], "exploring")
            
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
                    if nextGoalIndex >= len(mapping.noneVisitedList):
                        node_goal = None
                        break
                    elif mapping.checkGoalAnother(mapping.noneVisitedList[nextGoalIndex],"both") == True:
                        nextGoalIndex +=1
                    else:
                        node_goal = Node(mapping.noneVisitedList[nextGoalIndex])
                        break
                if node_goal != None:
                    mapping.updateGoals(currentPose, mapping.noneVisitedList[nextGoalIndex], robotInfo["id"])
                    mapping.updateStatus(robotInfo["id"], "moving")
                        
                    #node_goal = Node(mapping.noneVisitedList[0])
                    #goal = mapping.noneVisitedList[0]
                    print("planejar o caminho")
                    saveMap(mapping.structMap, "mappingPath")
                    path = PathPlanning(mapping,node_start,node_goal)
                    route = path.AStarAlgorithm()
                    print("Posição atual:" + str(robot.getAbsolutePosition(False)) + " do robô " + str(robotInfo["id"]))
                    print("Posição objetivo:" + str(node_goal.coord) + " do robô " + str(robotInfo["id"]))
                    print("Rota")
                    print(route)
                    
                    flagConflit = False
                    
                    while True:
                        #Ir até o lugar planejado
                        for index in reversed(range(1,len(route))):
                            if mapping.checkGoalAnother(route[index-1],"next") == False:
                                current_position = robot.getAbsolutePosition(False)
                                distance,angle = AuxiliarFunctions.CalcAngleDistance(current_position,route[index-1])
                                print("Distância:" + str(distance) + "// Angulo:" + str(angle))
                                robot.rotateTo(angle, velocity*0.8)
                                robot.moveFoward(distance, 0, velocity*3)
                                print("Posição atual e objetivo:")
                                print(robot.getAbsolutePosition(False))
                                print(route[index-1])
                            else:
                                flagConflit = True
                                break
                            
                        currentPosition = robot.getAbsolutePosition(False)
                        if math.sqrt( (route[0][0]-currentPosition[0] )**2 + (route[0][1]-currentPosition[1] )**2 ) < radius:
                            break
                        
                        if flagConflit == True:
                            print("deu conflito")
                

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
    
    # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    sim.simxGetPingTime(clientID)
    
    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)

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



mapping = Map(extRadius)


robot1str = "robot1"
robot2str = "robot2"

threading.Thread(target=Exploring,args=(1,19999)).start()
threading.Thread(target=Exploring,args=(2,19998)).start()
#Exploring('robot1.json')


time.sleep(200)




