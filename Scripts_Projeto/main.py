import sim
import time
import math
from Robots.Robot import Robot
from Lidar import Lidar
from Node import Node
from Map import Map
from AuxiliarFunctions import AuxiliarFunctions
from Save import saveCoord,saveEdge,saveMap,saveDebug,saveDebugCoord
from debug import logNodeInfo, logRobotInfo
import json
from PathPlanning import PathPlanning
import datetime
import threading

from rich.console import Console
from rich.table import Table

from Enums import *

from CommunicationAPI import CommunicationAPI

from Ordering import Ordering

def checkRobotExploring(mapping):
    for robot in mapping.statusMap:
        if robot["status"] != "stopping":
            return True
    return False



def orderBySeparate(listToOrder,robotsPosition):
    distanceList = []
    for node in listToOrder:
        distance = 0
        for robotCoord in robotsPosition:
            distance += math.sqrt( (node[0] - robotCoord[0])**2 +  (node[1] - robotCoord[1])**2)
        distanceList.append(distance)
    
    zipped_lists = zip(distanceList, listToOrder)
    sorted_zipped_lists = sorted(zipped_lists, reverse=True)

    sorted_list = [element for _, element in sorted_zipped_lists]

    return sorted_list
        


def Exploring(robotFile,comPort,radiusZone):
    #Definição de variáveis
    motorsObject = []
    #velocity = 0.7
    velocity = 0.35
    global mapping
    
    tInit = datetime.datetime.now()

    order = Order.Fifo
    orderN = Order.Maximum
    
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
    
    mapping.initStatusMap(robotObject["ID"], "stopping")
    
    currentPose = robot.getAbsolutePosition(False)
    print("Posição Inicial do robô " + str(robotObject["ID"]) + ": " + str(currentPose))
    mapping.addStepMap(robotObject["ID"], currentPose)
    mapping.initGoalsNode(robotObject["ID"], currentPose)
    mapping.initStatusMap(robotObject["ID"], "exploring")
    neighborhood,angles = robot.scanAround(0.7*velocity,mapping,currentPose)

    
    mapping.addMapPoint(currentPose,neighborhood,angles,robotObject["ID"])
    mapping.addVisitedNode(currentPose)
    string = "Coordenada adicionada pela condição 1:" + str(currentPose)
    saveDebugCoord(string,"debugLogCoord")
    mapping.addNoneVisitedNode(neighborhood)
    mapping.updateVisitedNode()

    '''
        Não pode haver nós a serem visitados e nem os visinhos podem estar explorando ou se mexendo
    '''
    while len(mapping.noneVisitedList) >0 or checkRobotExploring(mapping) == True:
        '''
            Se não existir um visinho a ser visitado, mas algum outro robô estiver caminhando ou explorando,
            o atual robô deverá esperar com o status de parado
        '''
        if len(mapping.noneVisitedList) == 0:
            #Varificar se o nó atual não está na passagem de ninguém
            mapping.updateStatus(robotObject["ID"], "stopping")
            time.sleep(1)
            string = "O robô " + str(robotObject["ID"]) + " parou porque está esperando aparecer nó não visitado"
            saveDebug(string)
            
        '''
            verificar se existe algum visinho disponível para ser visitado, ou seja, não estar na lista de nós visitados
            caso não, planejar o caminho para um nó não visitado
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
            neighborCoord = neighborhoodCoord[0]
            
            # Deslocar para a coordenada previamente incluida na lista, a fim de tentar minimizar o erro
            with lock:
                realDestiny = mapping.getCoordNoneVisitedList(neighborCoord)
            #print("Valor da coordenada real:" + str(realDestiny))
            diff = ((neighborCoord[0] - realDestiny[0])**2 + (neighborCoord[1] - realDestiny[1])**2 )**(1/2)
 
            string = 'Coordenada atual:' + str(currentPose) + '\n'
            string += "Coordenada neighbor index:"+ str(neighborCoord) + " // Coordenada real:"+ str(realDestiny) + '\n'
            
            distance,angle = AuxiliarFunctions.CalcAngleDistance(currentPose,realDestiny)
            
            #Definir o próximo objetivo/objetivo final, atualizar o status e visitá-lo
            with lock:
                mapping.updateGoals(currentPose,realDestiny, realDestiny, robotObject["ID"])
                mapping.updateStatus(robotObject["ID"], "moving")
                
            robot.rotateTo(angle, velocity*0.7)
            robot.moveFoward(distance, angle, velocity*3)
            
            with lock:
                currentPose = robot.getAbsolutePosition(False)
                string += "Coordenada de chegada:" + str(currentPose)
                saveDebugCoord(string,"debugLogDestiny")
                mapping.visitedNode(currentPose)
                string = "Coordenada adicionada pela condição 2:" + str(currentPose)
                saveDebugCoord(string,'debugLogCoord')
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
                
                mapping.updateStatus(robotObject["ID"], "exploring")
                
            neighborhood,angles = robot.scanAround(velocity*0.7,mapping,currentPose)
            with lock:
            #print("O robô " + str(robotObject["ID"]) + " vai adicionar " + str(len(neighborhood)) + " visinhos")
                mapping.addNoneVisitedNode(neighborhood)
                
                mapping.addMapPoint(currentPose,neighborhood,angles,robotObject["ID"])
            
        else:
            if len(mapping.noneVisitedList) > 0:
                #Escolher um nó para ir e planejar o caminho
                currentPose = robot.getAbsolutePosition(False)
                node_start = Node(currentPose)
                node_start.gcost = 0
                
                nextGoalIndex = 0
                '''
                if order == Order.Minimum:
                    with lock:
                        listDistance = orderByDistance(mapping.noneVisitedList,currentPose)
                elif order == Order.Maximum:
                    with lock:
                        robotsPosition = mapping.getRobotsPosition()
                        listDistance = orderBySeparate(mapping.noneVisitedList,robotsPosition)
                else:
                    listDistance = mapping.noneVisitedList
                '''
                listDistance  = getattr(ordering,str(order))(mapping.noneVisitedList)
                # Escolher o ponto futuro apenas se o mesmo não for objetivo de alguém

                with lock:
                    while True:
                        if nextGoalIndex >= len(listDistance):
                            node_goal = None
                            break
                        elif mapping.checkGoalAnother(listDistance[nextGoalIndex],"both",robotObject["ID"]) == True:
                            nextGoalIndex +=1
                        else:
                            node_goal = Node(listDistance[nextGoalIndex])
                            finalGoal = listDistance[nextGoalIndex]
                            break

                    
                    
                if node_goal != None:
                    if mapping.checkNoneVisitedList(mapping.noneVisitedList[nextGoalIndex]) == True:
                        #mapping.visitedNode(mapping.noneVisitedList[nextGoalIndex])
                        print("Nó adicionado pelo planejamento")
                        string = "A coordenada " + str(finalGoal) + " supostamente já foi adicionada e ainda é coordenada final"
                        saveDebug(string)
                    with lock:
                        mapping.updateGoals(currentPose,currentPose, finalGoal, robotObject["ID"])
                        mapping.updateStatus(robotObject["ID"], "moving")
                        

                    path = PathPlanning(mapping,node_start,node_goal)
                    route = path.AStarAlgorithm()
                    #print("Posição atual:" + str(currentPose) + " do robô " + str(robotObject["ID"]))
                    #print("Posição objetivo final:" + str(node_goal.coord) + " do robô " + str(robotObject["ID"]))
                    #print("Rota")
                    #print(route)
                    
                    flagConflit = False
                    

                    #Ir até o lugar planejado
                    for index in reversed(range(1,len(route))):
                        if mapping.checkGoalAnother(route[index-1],"next",robotObject["ID"]) == False:
                            try:
                                currentPose = robot.getAbsolutePosition(False)
                                distance,angle = AuxiliarFunctions.CalcAngleDistance(currentPose,route[index-1])

                                with lock:
                                    mapping.updateGoals(currentPose,route[index-1], finalGoal, robotObject["ID"])
                                    mapping.updateStatus(robotObject["ID"], "moving")
                                robot.rotateTo(angle, velocity*0.7)
                            
                                robot.moveFoward(distance, angle, velocity*3)
                                with lock:
                                    mapping.updateGoals(route[index-1],route[index-1], finalGoal, robotObject["ID"])
                                    mapping.addStepMap(robotObject["ID"], robot.getAbsolutePosition(False))
                            except:
                                print("Deu errado e caiu no except")
                                print([index-1])
                                print(nextGoalIndex)
                                print(len(mapping.noneVisitedList))
                        else:
                            conflitCoord = route[index-1]
                            flagConflit = True
                            break
                        
                    
                    if flagConflit == True:
                        '''
                            Trabalhar para verificar o tipo de conflito. Se ele o robô do conflito
                            só puder ir para onde o atual robô está, ver como resolver.
                        '''
                        with lock:
                            mapping.updateStatus(robotObject["ID"], "conflit")
                        currentPose = robot.getAbsolutePosition(False)
                        '''
                            Se o outro robô estiver se mexendo ou explorando, o robô atual deverá esperar. Caso ele esteja em conflito, deverá ser realizada
                            alguma ação com o objetivo de dar passagem
                        '''

                        string = "Deu conflito na execução do robô " + str(robotObject["ID"])
                        saveDebug(string)
                        
                        if mapping.getStatusRobot(conflitCoord) != "conflit":
                            string = "Deu conflito que será resolvido apenas esperando o outro robô terminar de caminhar ou explorar - " + str(robotObject["ID"])
                            saveDebug(string)
                            time.sleep(3)
                        else:
                            string = "Deu conflito que será resolvido com a movimentação de um dos robôs envolvidos"
                            saveDebug(string)
                            '''
                                Caso o robô atual tenha mais casas para andar que o outro, ele deve dar passagem. 
                                Caso não, esperar o outro dar passagem.
                                Dar passagem implica em procurar um nó visinho disponível e ir para ele
                            '''
                            currentPose = robot.getAbsolutePosition(False)
                            if mapping.checkAdjNumber(currentPose) >= mapping.checkAdjNumber(conflitCoord):
                                string = "O robô " + str(robotObject["ID"]) + " vai dar passagem"
                                saveDebug(string)
                                print("O robô " + str(robotObject["ID"]) + " tem mais casas para andar")
                                print("Escolher um lugar para se mexer que não colida com ninguém")
                                neighbor = mapping.getFreeNeighbor(currentPose,robotObject["ID"])
                                print("Saindo do meio")
                                distance,angle = AuxiliarFunctions.CalcAngleDistance(currentPose,neighbor)
                                print("Distância:" + str(distance) + "// Angulo:" + str(angle))
                                with lock:
                                    mapping.updateGoals(currentPose,neighbor, neighbor, robotObject["ID"])
                                    mapping.updateStatus(robotObject["ID"], "moving")
                                robot.rotateTo(angle, velocity*0.7)
                                #print("Movimentação 3 do robô: " + str(robotObject["ID"]))
                                robot.moveFoward(distance, angle, velocity*3)
                                
                                currentPose = robot.getAbsolutePosition(False)
                                with lock:
                                    mapping.updateGoals(neighbor,neighbor, neighbor, robotObject["ID"])
                                    mapping.addStepMap(robotObject["ID"], currentPose)
                            
                            else:
                                time.sleep(3)
                                string = "O robô " + str(robotObject["ID"]) + " vai esperar o espaço"
                                saveDebug(string)
                                
                        
                        
                    else:
                        currentPose = robot.getAbsolutePosition(False)
                        #print("Terminou com posição atual e objetivo:")
                        #print(currentPose)
                        #print(route[0])

                        with lock:
                            mapping.visitedNode(currentPose)
                            string = "Coordenada adicionada pela condição 3:"+str(currentPose)
                            saveDebugCoord(string,"debugLogCoord")
                            saveCoord(mapping.visitedList,"coord.csv")
                            saveCoord(mapping.noneVisitedList,"noneVisited.csv")
                        
                        neighborhood,angles = robot.scanAround(0.7*velocity,mapping,currentPose)
                        with lock:
                            mapping.addNoneVisitedNode(neighborhood)
                            
                            mapping.addMapPoint(currentPose,neighborhood,angles,robotObject["ID"])
                        
                elif mapping.checkGoalAnother(robot.getAbsolutePosition(False),"next",robotObject["ID"]) == True:
                    currentPose = robot.getAbsolutePosition(False)
                    string = "O robô " + str(robotObject["ID"]) + " vai dar passagem na condição de única alternativa"
                    saveDebug(string)
                    print("Escolher um lugar para se mexer que não colida com ninguém")
                    with lock:
                        neighbor = mapping.getFreeNeighbor(currentPose,robotObject["ID"])
                        mapping.updateGoals(currentPose,neighbor, neighbor, robotObject["ID"])
                    print("Saindo do meio")
                    distance,angle = AuxiliarFunctions.CalcAngleDistance(currentPose,neighbor)
                    print("Distância:" + str(distance) + "// Angulo:" + str(angle))
                    
                    robot.rotateTo(angle, velocity*0.7)
                    robot.moveFoward(distance, angle, velocity*3)
                    currentPose = robot.getAbsolutePosition(False)
                    with lock:
                        mapping.updateGoals(neighbor,neighbor, neighbor, robotObject["ID"])
                        mapping.addStepMap(robotObject["ID"], currentPose)
                    
            

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


threading.Thread(target=Exploring,args=(1,19999,radius)).start()
threading.Thread(target=Exploring,args=(2,19998,radius)).start()
threading.Thread(target=Exploring,args=(3,19997,radius)).start()






