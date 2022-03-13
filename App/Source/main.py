import sim
import time
import math
import json
import datetime
import threading

from random import random
from Robots.Robot import Robot
from Lidar import Lidar
from Node import Node
from Map import Map
from AuxiliarFunctions import AuxiliarFunctions
from Save import saveCoord,saveEdge,saveMap,saveDebug,saveDebugCoord
from debug import logNodeInfo, logRobotInfo
from PathPlanning import PathPlanning


from rich.console import Console
from rich.table import Table

from Enums import *

from CommunicationAPI import CommunicationAPI

from Ordering import Ordering

def checkRobotExploring(mapping):
    for robot in mapping.statusMap:
        if robot["status"] != ExplorationMode.Standby:
            return True
    return False

def RandomRuleLinear(ordering,neighborhoodCoord,moment):
    prob = random()
    if prob < 0.33:
        orderN = Order.Fifo
    elif prob < 0.66 and prob >= 0.33:
        orderN = Order.Minimum
    else:
        orderN = Order.Maximum
    string = "Momento da escolhda: " + str(moment) + " Regra escolhida: " + str(orderN)
    saveDebugCoord(string,"RandomRule")
    return getattr(ordering, str(orderN))(neighborhoodCoord)


def GiveAway(robot, mapping, velocity):
    with lock:
        currPosition = robot.getAbsolutePosition(False)
        neighbor = mapping.GetFreeNeighbor(currPosition, robot.robotInfos["ID"])
        
        if neighbor != None:
            
            distance, angle = AuxiliarFunctions.CalcAngleDistance(currPosition, neighbor)

            mapping.updateGoals(currPosition, neighbor, neighbor, robot.robotInfos["ID"])
    
            robot.rotateTo(angle, velocity*0.7)

            robot.moveFoward(distance, angle, velocity*3)

            #currPosition = robot.getAbsolutePosition(False)

            mapping.updateGoals(neighbor, neighbor, neighbor, robot.robotInfos["ID"])

            mapping.addStepMap(robot.robotInfos["ID"], currPosition)
            #Fazer o update do passado dado no mapa de passos
            return True
            
        else:
            return False

def WaitConflitResolution(robot, mapping, coordConflit):
    while True:
        if mapping.checkGoalAnother(coordConflit, "both", robot.robotInfos["ID"]) == False:
            return

def WaitFreeNeighborhood(robot, mapping):
    currPosition = robot.getAbsolutePosition(False)
    while True:
        if mapping.checkAdjNumber(currPosition) > 0:
            return

        


def Exploring(robotFile,comPort,radiusZone):
    #Definição de variáveis
    motorsObject = []
    #velocity = 0.7
    velocity = 0.35
    global mapping
    
    tInit = datetime.datetime.now()

    orderN = Order.Maximum
    order = Order.Fifo    
    
    #Pegar informações do json do robô
    fileJson = open("Schemas/robot.json")
    robotString =  "Robot" + str(robotFile) 
    robotInfo = json.load(fileJson)
    
    comm = CommunicationAPI('127.0.0.1', comPort)
    
    #Pegar o objeto do robô
    returnCode,khepera=sim.simxGetObjectHandle(comm.clientId,robotInfo[robotString]["robotBody"],sim.simx_opmode_blocking)

    #Pegar objeto dos motores manipuláveis
    for i in range(2):
        returnCode,handle=sim.simxGetObjectHandle(comm.clientId,robotInfo[robotString]["motorsName"][i],sim.simx_opmode_blocking)
        motorsObject.append(handle)
    
    # Criar o objeto robô
    robotObject = {
        "ID":           robotInfo[robotString]["id"],
        "leftMotor":    motorsObject[0],
        "rightMotor":   motorsObject[1],
        "robot":        khepera  
        }
    
    #Criar objetos do acelerômetro, Lidar e giroscópio
    lidar = Lidar(robotInfo[robotString]["lidar"],comm.clientId,robotObject["ID"])
    
    #Criar o objeto do robô
    robot = Robot(comm,robotObject,lidar,radiusZone)

    ordering = Ordering(mapping,robot)
    
    #Tempo para estabelecer conexões com o servidor
    time.sleep(0.2)
    

    robot.rotateTo(30,5)
    #Teste de movimentação
    #robot.moveFoward(8,0, 10)
    
    time.sleep(20)
    
    
    with lock:
        mapping.initStatusMap(robotObject["ID"], ExplorationMode.Standby)
        
        currentPose = robot.getAbsolutePosition(False)
    
        mapping.addStepMap(robotObject["ID"], currentPose)
        mapping.initGoalsNode(robotObject["ID"], currentPose)
        mapping.initStatusMap(robotObject["ID"], ExplorationMode.Exploring)

    neighborhood,angles = robot.scanAround(0.7*velocity,mapping,currentPose)

    with lock:
        mapping.addMapPoint(currentPose,neighborhood,angles,robotObject["ID"])
        mapping.addVisitedNode(currentPose)
        string = "Coordenada adicionada pela condicao 1:" + str(currentPose)
        saveDebugCoord(string,"EventLog")
        mapping.addNoneVisitedNode(neighborhood)
        mapping.updateVisitedNode()

    '''
        Não pode haver nós a serem visitados e nem os visinhos podem estar explorando ou se mexendo
    '''
    while len(mapping.noneVisitedList) > 0 or checkRobotExploring(mapping) == True:
        '''
            Se não existir um visinho a ser visitado, mas algum outro robô estiver caminhando ou explorando,
            o atual robô deverá esperar com o status de parado
        '''
        if len(mapping.noneVisitedList) == 0:
            #Verificar se o nó atual está na passagem de outro agente
            if mapping.checkGoalAnother(robot.getAbsolutePosition(False), "next", robotObject["ID"]) == True:
                GiveAway(robot, mapping, velocity)

            mapping.updateStatus(robotObject["ID"], ExplorationMode.Standby)
            #time.sleep(1)
            #string = "O robô " + str(robotObject["ID"]) + " parou porque está esperando aparecer nó não visitado"
            #saveDebug(string)
            continue
            
        '''
            Verificar se existe algum visinho disponível para ser visitado, ou seja, não estar na lista de nós visitados.
            Caso não haja, planejar o caminho para um nó não visitado
        '''
        with lock:
            try:
                #neighborhoodCoord,neighborhoodAngle = mapping.checkAvailability(neighborhood,angles,robotObject["ID"])
                neighborhoodCoord = mapping.checkAvailability(neighborhood,angles,robotObject["ID"])
            except Exception:
                neighborhoodCoord = []
       
        if len(neighborhoodCoord) !=  0 :
        
            #Visitar o nó e reordenar de acordo com a regra vigente
            print("Nó não visitado do robô:" + str(robotObject["ID"]))
            neighborhoodCoord  = getattr(ordering, str(orderN))(neighborhoodCoord)
            #neighborhoodCoord = RandomRuleLinear(ordering,neighborhoodCoord,"Regra 1")
            neighborCoord = neighborhoodCoord[0]
            
            # Deslocar para a coordenada previamente incluida na lista, a fim de tentar minimizar o erro
            with lock:
                realDestiny = mapping.getCoordNoneVisitedList(neighborCoord)
                currentPose = robot.getAbsolutePosition(False)
            #print("Valor da coordenada real:" + str(realDestiny))
            #diff = ((neighborCoord[0] - realDestiny[0])**2 + (neighborCoord[1] - realDestiny[1])**2 )**(1/2)
 
            #string = 'Coordenada atual:' + str(currentPose) + '\n'
            #string += "Coordenada neighbor index:"+ str(neighborCoord) + " // Coordenada real:"+ str(realDestiny) + '\n'
            
            distance,angle = AuxiliarFunctions.CalcAngleDistance(currentPose,realDestiny)
            
            '''
                Definir o próximo objetivo/objetivo final, atualizar o status e visitá-lo.
                A atualização do status só deve acontecer a partir do momento em que o agente já estiver pronto para sair,
                evitando que haja uma colisão no momento da rotação
            '''
            #
            with lock:
                mapping.updateGoals(currentPose, realDestiny, realDestiny, robotObject["ID"])
                mapping.updateStatus(robotObject["ID"], ExplorationMode.Exploring)

            robot.rotateTo(angle, velocity*0.7)

            robot.moveFoward(distance, angle, velocity*3)
            
            with lock:
                currentPose = robot.getAbsolutePosition(False)
                #string += "Coordenada de chegada:" + str(currentPose)
                #saveDebugCoord(string,"debugLogDestiny")
                mapping.visitedNode(currentPose)
                string = "Coordenada adicionada pela condicao 2:" + str(currentPose)
                saveDebugCoord(string,'EventLog')
                mapping.updateGoals(currentPose,realDestiny,realDestiny,robotObject["ID"])

                '''
                    Uma vez que se chegou no nó não visitado, deve-se escanear em volta, e adicionar os visinhos, não visitados,
                    a lista de nós não visitados, atualizar o nó atual do mapa e dizer que o nó atual é filho do nó anterior.
                    Também deve-se adiconar o nó atual a lista de visitados, assim como dizer que o atual nó, visinho do pai, foi visitado
                '''

                mapping.addStepMap(robotObject["ID"], currentPose)
                #print("Coordenadas destino: " + str(neighborCoord) )
                #print("Coordenada final:" + str(currentPose) + " do robô " + str(robotObject["ID"]))

                saveCoord(mapping.visitedList,"coord.csv")
                saveCoord(mapping.noneVisitedList,"noneVisited.csv")
                
                mapping.updateStatus(robotObject["ID"], ExplorationMode.Exploring)
                
            neighborhood,angles = robot.scanAround(velocity*0.7, mapping,currentPose)

            with lock:
                mapping.addNoneVisitedNode(neighborhood)
                mapping.addMapPoint(currentPose,neighborhood,angles,robotObject["ID"])
            
        else:
            if len(mapping.noneVisitedList) > 0:
                #Escolher um nó para ir e planejar o caminho
                currentPose = robot.getAbsolutePosition(False)
                node_start = Node(currentPose)
                node_start.gcost = 0
                
                nextGoalIndex = 0

                #listDistance = RandomRuleLinear(ordering,mapping.noneVisitedList,"Regra 2")
                listDistance  = getattr(ordering,str(order))(mapping.noneVisitedList)
                # Escolher o ponto futuro apenas se o mesmo não for objetivo de alguém

                node_goal = None
                while True:
                    if nextGoalIndex >= len(listDistance):
                        node_goal = None
                        break
                    elif mapping.checkGoalAnother(listDistance[nextGoalIndex],"both", robotObject["ID"]) == True:
                        nextGoalIndex +=1
                    else:
                        node_goal = Node(listDistance[nextGoalIndex])
                        finalGoal = listDistance[nextGoalIndex]

                    if node_goal != None:
                        path = PathPlanning(mapping,node_start,node_goal)
                        route,executionFlag = path.AStarAlgorithm()
                        if executionFlag == True:
                            break
                        nextGoalIndex +=1
                        node_goal = None
                            

                if node_goal != None:
                    with lock:
                        if mapping.checkNoneVisitedList(mapping.noneVisitedList[nextGoalIndex]) == True:
                            print("Nó a ser visitado pelo planejamento: {}".format(mapping.noneVisitedList[nextGoalIndex]))
                            string = "A coordenada " + str(finalGoal) + " supostamente já foi adicionada e ainda é coordenada final"
                            saveDebug(string)
                    
                            mapping.updateGoals(currentPose,currentPose, finalGoal, robotObject["ID"])
                            mapping.updateStatus(robotObject["ID"], ExplorationMode.Exploring)
                        

                    flagConflit = False
                    
                    #Ir até o lugar planejado
                    for index in reversed(range(1,len(route))):
                        if mapping.checkGoalAnother(route[index-1],"next", robotObject["ID"]) == False:
                            try:
                                currentPose = robot.getAbsolutePosition(False)
                                distance,angle = AuxiliarFunctions.CalcAngleDistance(currentPose,route[index-1])

                                with lock:
                                    mapping.updateGoals(currentPose,route[index-1], finalGoal, robotObject["ID"])
                                    mapping.updateStatus(robotObject["ID"], ExplorationMode.Exploring)

                                robot.rotateTo(angle, velocity*0.7)

                                robot.moveFoward(distance, angle, velocity*3)

                                with lock:
                                    mapping.updateGoals(route[index-1],route[index-1], finalGoal, robotObject["ID"])
                                    mapping.addStepMap(robotObject["ID"], robot.getAbsolutePosition(False))

                            except Exception as inst:
                                errorMessage = inst.args[0]
                                print(errorMessage)
                                saveDebug(errorMessage)
                        else:
                            conflitCoord = route[index-1]
                            flagConflit = True
                            mapping.updateStatus(robotObject["ID"], ExplorationMode.Conflit)
                            break
                        
 
                    if flagConflit == False:
                        currentPose = robot.getAbsolutePosition(False)

                        with lock:
                            mapping.visitedNode(currentPose)
                            string = "Coordenada adicionada pela condição 3:"+str(currentPose)
                            saveDebugCoord(string,"EventLog")
                            saveCoord(mapping.visitedList,"coord.csv")
                            saveCoord(mapping.noneVisitedList,"noneVisited.csv")
                        
                        neighborhood,angles = robot.scanAround(0.7*velocity,mapping,currentPose)
                        with lock:
                            mapping.addNoneVisitedNode(neighborhood)
                            mapping.addMapPoint(currentPose,neighborhood,angles,robotObject["ID"])
                else:
                    #currentPose = robot.getAbsolutePosition(False)
                    with lock:
                        mapping.updateGoals(currentPose,currentPose, currentPose, robotObject["ID"])
                        mapping.updateStatus(robotObject["ID"], ExplorationMode.Standby)

            if flagConflit == True:
                '''
                    Uma vez que foi detectado conflito, é necessário verificar o tipo de conflito.
                    Conflitos possíveis:
                        1. Um agente na passagem e ele está apenas de passagem:
                            Resolução: Esperar
                        2. Agentes simultaneamente estão bloqueado o caminho
                            Resolução: O agente de maior quantidade de vizinhos disponíveis deverá dar passagem
                        3. O agente não possíveis vizinhos disponíveis.
                            Resolução: Esperar
                        4. O outro agente está em standby
                            Resolução: Esperar
                '''
                currentPose = robot.getAbsolutePosition(False)
                if mapping.checkAdjNumber(currentPose) == 0:
                    continue
                elif mapping.getStatusRobot(conflitCoord) == ExplorationMode.Standby:
                    continue
                elif mapping.getStatusRobot(conflitCoord) == ExplorationMode.Exploring:
                    continue
                elif mapping.getStatusRobot(conflitCoord) == ExplorationMode.Blocked:
                    executionFlag = GiveAway(robot, mapping, velocity)

                    if executionFlag == False:
                        mapping.updateStatus(robotObject["ID"], ExplorationMode.Blocked)
                    continue
                #elif mapping.getStatusRobot(conflitCoord) != ExplorationMode.Conflit and mapping.getStatusRobot(conflitCoord) != ExplorationMode.Conflit:
                    #string = "Deu conflito que será resolvido apenas esperando o outro robô terminar de caminhar ou explorar - " + str(robotObject["ID"])
                    #saveDebug(string)
                    #time.sleep(3)
                    #continue
                else:
                    #currentPose = robot.getAbsolutePosition(False)
                    if mapping.checkAdjNumber(currentPose) >= mapping.checkAdjNumber(conflitCoord):
                        
                        GiveAway(robot, mapping, velocity)
                        
                        
                        string = "O robô " + str(robotObject["ID"]) + " vai dar passagem"
                        saveDebug(string)
                        print(string)

                        '''
                        with lock:
                            mapping.updateGoals(neighbor,neighbor, neighbor, robotObject["ID"])
                            mapping.addStepMap(robotObject["ID"], currentPose)
                        '''
                    
                    else:
                        time.sleep(3)
                        string = "O robô " + str(robotObject["ID"]) + " vai esperar o espaço"
                        saveDebug(string)

            '''
                Resolução de conflito passivo, que acorre quando um agente está parado e outro tenta acessar sua célula. 
                Nesse caso, a prioridade será dada ao agente que está se movendo, a menos que não seja possível se mover
            '''

            if mapping.checkGoalAnother(robot.getAbsolutePosition(False), "next", robotObject["ID"]) == True:
                currentPose = robot.getAbsolutePosition(False)
                string = "O robô " + str(robotObject["ID"]) + " vai dar passagem na condição de única alternativa"
                saveDebug(string)
                print("Escolher um lugar para se mexer que não colida com ninguém")

                executionFlag = GiveAway(robot, mapping, velocity)

                if executionFlag == False:
                    mapping.updateStatus(robotObject["ID"], ExplorationMode.Blocked)


                '''
                with lock:
                    neighbor = mapping.GetFreeNeighbor(currentPose,robotObject["ID"])
                    mapping.updateGoals(currentPose,neighbor, neighbor, robotObject["ID"])
                print("Saindo do meio")
                distance,angle = AuxiliarFunctions.CalcAngleDistance(currentPose,neighbor)
                print("Distância:" + str(distance) + "// Angulo:" + str(angle))
                
                robot.rotateTo(angle, velocity*0.7)
                robot.moveFoward(distance, angle, velocity*3)
                '''
                '''
                currentPose = robot.getAbsolutePosition(False)

                with lock:
                    mapping.updateGoals(neighbor, neighbor, neighbor, robotObject["ID"])
                    mapping.addStepMap(robotObject["ID"], currentPose)
                '''

            

                    
            

    saveMap(mapping.structMap, "map")
    tFinal = datetime.datetime.now()
    deltaT = tFinal- tInit
    print(deltaT)
    #saveEdge(mapping.edgeMap,'arestas')
    print("Terminou")
    
    # # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    sim.simxGetPingTime(comm.clientId)
    
    # # Now close the connection to CoppeliaSim:
    comm.Dispose()

print ('Program started')

#Variáveis do projeto
#radius = 0.15
radius = 0.25
extRadius = radius/math.cos(math.pi/6)

contagem = 0

mapping = Map(extRadius)

CommunicationAPI.Clear()

lock = threading.Lock()

Exploring(1,19999,radius)

#threading.Thread(target=Exploring,args=(1,19999,radius)).start()
#threading.Thread(target=Exploring,args=(2,19998,radius)).start()
#threading.Thread(target=Exploring,args=(3,19997,radius)).start()