import sim
import time
import math
from Robot import Robot
from Lidar import Lidar
from Node import Node
from Map import Map
from AuxiliarFunctions import AuxiliarFunctions
from Save import saveCoord,saveEdge,saveMap,saveDebug,saveDebugCoord
from debug import logNodeInfo
import json
from PathPlanning import PathPlanning
import datetime
import threading

def checkRobotExploring(mapping):
    for robot in mapping.statusMap:
        if robot["status"] != "stopping":
            return True
    return False

def orderByDistance(listToOrder,point):
    distanceList = []
    for node in listToOrder:
        distance = math.sqrt( (node[0] - point[0])**2 +  (node[1] - point[1])**2)
        distanceList.append(distance)
    
    zipped_lists = zip(distanceList, listToOrder)
    sorted_zipped_lists = sorted(zipped_lists)

    sorted_list1 = [element for _, element in sorted_zipped_lists]
    
    #print(sorted_list1)
    return sorted_list1

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
    order = 'maximum'
    
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
    robot = Robot(clientID,robotInfo["id"],motorsObject,khepera,lidar,radiusZone)
    
    #Tempo para estabelecer conexões com o servidor
    time.sleep(0.2)
    
    mapping.initStatusMap(robotInfo["id"], "stopping")
    
    currentPose = robot.getAbsolutePosition(False)
    print("Posição Inicial do robô " + str(robotInfo["id"]) + ": " + str(currentPose))
    mapping.addStepMap(robotInfo["id"], currentPose)
    mapping.initGoalsNode(robotInfo["id"], currentPose)
    mapping.initStatusMap(robotInfo["id"], "exploring")
    neighborhood,angles = robot.scanAround(0.7*velocity,mapping,currentPose)

    
    mapping.addMapPoint(currentPose,neighborhood,angles,robotInfo["id"])
    mapping.addVisitedNode(currentPose)
    string = "Coordenada adicionada pela condição 1:"+str(currentPose)
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
            mapping.updateStatus(robotInfo["id"], "stopping")
            time.sleep(1)
            string = "O robô " + str(robotInfo["id"]) + " parou porque está esperando aparecer nó não visitado"
            saveDebug(string)
            
        '''
            verificar se existe algum visinho disponível para ser visitado, ou seja, não estar na lista de nós visitados
            caso não, planejar o caminho para um nó não visitado
        '''
        with lock:
            try:
                neighborCoord,neighborAngle = mapping.checkAvailability(neighborhood,angles,robotInfo["id"])
            except Exception:
                neighborCoord = None
        
        if neighborCoord != None:
            #Visitar o nó 
            print("Nó não visitado do robô:" + str(robotInfo["id"]))
            
            # Deslocar para a coordenada previamente incluida na lista, a fim de tentar minimizar o erro
            #print("Valor da coordenada do check:" + str(neighborCoord))
            #flag = mapping.checkVisitedStruct(neighborCoord)
            #print("O valor da flag: " + str(flag) + " do robô:" + str(robotInfo["id"]))
            with lock:
                realDestiny = mapping.getCoordNoneVisitedList(neighborCoord)
            #print("Valor da coordenada real:" + str(realDestiny))
            diff = ((neighborCoord[0] - realDestiny[0])**2 + (neighborCoord[1] - realDestiny[1])**2 )**(1/2)
            '''
            if diff>radius/4:
                realDestiny = neighborCoord
                #currentPose = realPose
                print("---------------------- Diferença que fez dar errado ----------------------")
                print(diff)
                string = "Destino do robô " + str(robotInfo["id"]) + " não coincide com o destino real"
                saveDebug(string)
            
            print("Suposta posição do robô: " + str(currentPose))
            realPose = robot.getAbsolutePosition(False)
            print("Posição real do robô: " + str(realPose))
            diff = ((currentPose[0] - realPose[0])**2 + (currentPose[1] - realPose[1])**2 )**(1/2)
            #print(diff)
            if diff>radius/4:
                currentPose = realPose
                print("--------------Diferença que fez dar errado 2 ----------------")
                print(diff)
                string = "Posição do robô " + str(robotInfo["id"]) + " não coincide no currentPose com a posição real"
                saveDebug(string)
            '''  
            #currentPose = robot.getAbsolutePosition(False)
            string = 'Coordenada atual:' + str(currentPose) + '\n'
            string += "Coordenada neighbor index:"+ str(neighborCoord) + " // Coordenada real:"+ str(realDestiny) + '\n'
            
            distance,angle = AuxiliarFunctions.CalcAngleDistance(currentPose,realDestiny)
            
            #Definir o próximo objetivo/objetivo final, atualizar o status e visitá-lo
            with lock:
                mapping.updateGoals(currentPose,realDestiny, realDestiny, robotInfo["id"])
                mapping.updateStatus(robotInfo["id"], "moving")
                
            robot.rotateTo(angle, velocity*0.7)
            #print("Movimentação 1 do robô: " + str(robotInfo["id"]))
            robot.moveFoward(distance, angle, velocity*3)
            
            with lock:
                currentPose = robot.getAbsolutePosition(False)
                string += "Coordenada de chegada:" + str(currentPose)
                saveDebugCoord(string,"debugLogDestiny")
                mapping.visitedNode(currentPose)
                string = "Coordenada adicionada pela condição 2:"+str(currentPose)
                saveDebugCoord(string,'debugLogCoord')
                mapping.updateGoals(currentPose,realDestiny,realDestiny,robotInfo["id"])
                
                '''
                robot.rotateTo(defineAngle(neighborAngle), velocity*0.8)
                distParent = robot.moveFoward(2*radius, 0, velocity*3)
                '''
                #print("Distancia deslocada: " + str(distParent) + " pelo robô "+ str(robotInfo["id"]))
                
                '''
                    Uma vez que se chegou no nó não visitado, deve-se escanear em volta, e adicionar os visinhos, não visitados,
                    a lista de nós não visitados, atualizar o nó atual do mapa e dizer que o nó atual é filho do nó anterior.
                    Também deve-se adiconar o nó atual a lista de visitados, assim como dizer que o atual nó, visinho do pai, foi visitado
                '''
                
                
                mapping.addStepMap(robotInfo["id"], currentPose)
                #print("Coordenadas destino: " + str(neighborCoord) )
                #print("Coordenada final:" + str(currentPose) + " do robô " + str(robotInfo["id"]))
                
                
                saveCoord(mapping.visitedList,"coord.csv")
                saveCoord(mapping.noneVisitedList,"noneVisited.csv")
                
                mapping.updateStatus(robotInfo["id"], "exploring")
                
            neighborhood,angles = robot.scanAround(velocity*0.7,mapping,currentPose)
            with lock:
            #print("O robô " + str(robotInfo["id"]) + " vai adicionar " + str(len(neighborhood)) + " visinhos")
                mapping.addNoneVisitedNode(neighborhood)
                
                mapping.addMapPoint(currentPose,neighborhood,angles,robotInfo["id"])
            
        else:
            if len(mapping.noneVisitedList) > 0:
                #Escolher um nó para ir e planejar o caminho
                currentPose = robot.getAbsolutePosition(False)
                node_start = Node(currentPose)
                node_start.gcost = 0
                
                nextGoalIndex = 0
                
                if order == 'minimum':
                    with lock:
                        listDistance = orderByDistance(mapping.noneVisitedList,currentPose)
                elif order == 'maximum':
                    with lock:
                        robotsPosition = []
                        for nodes in mapping.goalsMap:
                            robotsPosition.append(nodes["currentPosition"])
                        listDistance = orderBySeparate(mapping.noneVisitedList,robotsPosition)
                    
                # Escolher o ponto futuro apenas se o mesmo não for objetivo de alguém
                with lock:
                    while True:
                        if order == 'fifo':
                            #if mapping.checkVisitedCoord(mapping.noneVisitedList[nextGoalIndex]) == True:
                                #mapping.visitedNode(mapping.noneVisitedList[nextGoalIndex])
                                #print("Nó adicionado pelo planejamento")
                            #else:
                            if nextGoalIndex >= len(mapping.noneVisitedList):
                                node_goal = None
                                break
                            elif mapping.checkGoalAnother(mapping.noneVisitedList[nextGoalIndex],"both",robotInfo["id"]) == True:
                                nextGoalIndex +=1
                            else:
                                node_goal = Node(mapping.noneVisitedList[nextGoalIndex])
                                finalGoal = mapping.noneVisitedList[nextGoalIndex]
                                break
                        else:
                            if nextGoalIndex >= len(mapping.noneVisitedList):
                                node_goal = None
                                break
                            elif mapping.checkGoalAnother(listDistance[nextGoalIndex],"both",robotInfo["id"]) == True:
                                nextGoalIndex +=1
                            else:
                                node_goal = Node(listDistance[nextGoalIndex])
                                finalGoal = listDistance[nextGoalIndex]
                                break

                    
                    
                if node_goal != None:
                    if mapping.checkNoneVisitedList2(mapping.noneVisitedList[nextGoalIndex]) == True:
                        #mapping.visitedNode(mapping.noneVisitedList[nextGoalIndex])
                        print("Nó adicionado pelo planejamento")
                        string = "A coordenada " + str(finalGoal) + " supostamente já foi adicionada e ainda é coordenada final"
                        saveDebug(string)
                    with lock:
                        mapping.updateGoals(currentPose,currentPose, finalGoal, robotInfo["id"])
                        mapping.updateStatus(robotInfo["id"], "moving")
                        
                    #print("planejar o caminho")
                    #saveMap(mapping.structMap, "mappingPath")
                    path = PathPlanning(mapping,node_start,node_goal)
                    route = path.AStarAlgorithm()
                    #print("Posição atual:" + str(currentPose) + " do robô " + str(robotInfo["id"]))
                    #print("Posição objetivo final:" + str(node_goal.coord) + " do robô " + str(robotInfo["id"]))
                    #print("Rota")
                    #print(route)
                    
                    flagConflit = False
                    

                    #Ir até o lugar planejado
                    for index in reversed(range(1,len(route))):
                        if mapping.checkGoalAnother(route[index-1],"next",robotInfo["id"]) == False:
                            try:
                                currentPose = robot.getAbsolutePosition(False)
                                distance,angle = AuxiliarFunctions.CalcAngleDistance(currentPose,route[index-1])
                                #print("Distância:" + str(distance) + "// Angulo:" + str(angle))
                                with lock:
                                    mapping.updateGoals(currentPose,route[index-1], finalGoal, robotInfo["id"])
                                    mapping.updateStatus(robotInfo["id"], "moving")
                                robot.rotateTo(angle, velocity*0.7)
                                
                                #print("Movimentação 2 do robô: " + str(robotInfo["id"]))
                                robot.moveFoward(distance, angle, velocity*3)
                                with lock:
                                    mapping.updateGoals(route[index-1],route[index-1], finalGoal, robotInfo["id"])
                                    mapping.addStepMap(robotInfo["id"], robot.getAbsolutePosition(False))
                                #print("Posição atual e objetivo:")
                                #print(robot.getAbsolutePosition(False))
                               # print(route[index-1])
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
                            mapping.updateStatus(robotInfo["id"], "conflit")
                        currentPose = robot.getAbsolutePosition(False)
                        '''
                            Se o outro robô estiver se mexendo ou explorando, o robô atual deverá esperar. Caso ele esteja em conflito, deverá ser realizada
                            alguma ação com o objetivo de dar passagem
                        '''
                        #print("deu conflito")
                        string = "Deu conflito na execução do robô " + str(robotInfo["id"])
                        saveDebug(string)
                        
                        if mapping.getStatusRobot(conflitCoord) != "conflit":
                            string = "Deu conflito que será resolvido apenas esperando o outro robô terminar de caminhar ou explorar - " + str(robotInfo["id"])
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
                                string = "O robô " + str(robotInfo["id"]) + " vai dar passagem"
                                saveDebug(string)
                                print("O robô " + str(robotInfo["id"]) + " tem mais casas para andar")
                                print("Escolher um lugar para se mexer que não colida com ninguém")
                                neighbor = mapping.getFreeNeighbor(currentPose,robotInfo["id"])
                                print("Saindo do meio")
                                distance,angle = AuxiliarFunctions.CalcAngleDistance(currentPose,neighbor)
                                print("Distância:" + str(distance) + "// Angulo:" + str(angle))
                                with lock:
                                    mapping.updateGoals(currentPose,neighbor, neighbor, robotInfo["id"])
                                    mapping.updateStatus(robotInfo["id"], "moving")
                                robot.rotateTo(angle, velocity*0.7)
                                #print("Movimentação 3 do robô: " + str(robotInfo["id"]))
                                robot.moveFoward(distance, angle, velocity*3)
                                
                                currentPose = robot.getAbsolutePosition(False)
                                with lock:
                                    mapping.updateGoals(neighbor,neighbor, neighbor, robotInfo["id"])
                                    mapping.addStepMap(robotInfo["id"], currentPose)
                            
                            else:
                                time.sleep(3)
                                string = "O robô " + str(robotInfo["id"]) + " vai esperar o espaço"
                                saveDebug(string)
                                
                        
                        
                    else:
                        currentPose = robot.getAbsolutePosition(False)
                        #print("Terminou com posição atual e objetivo:")
                        #print(currentPose)
                        #print(route[0])
                        
                        #currentPose = robot.getAbsolutePosition(False)
                        with lock:
                            mapping.visitedNode(currentPose)
                            string = "Coordenada adicionada pela condição 3:"+str(currentPose)
                            saveDebugCoord(string,"debugLogCoord")
                            saveCoord(mapping.visitedList,"coord.csv")
                            saveCoord(mapping.noneVisitedList,"noneVisited.csv")
                        
                        neighborhood,angles = robot.scanAround(0.7*velocity,mapping,currentPose)
                        with lock:
                            mapping.addNoneVisitedNode(neighborhood)
                            
                            mapping.addMapPoint(currentPose,neighborhood,angles,robotInfo["id"])
                        
                elif mapping.checkGoalAnother(robot.getAbsolutePosition(False),"next",robotInfo["id"]) == True:
                    currentPose = robot.getAbsolutePosition(False)
                    string = "O robô " + str(robotInfo["id"]) + " vai dar passagem na condição de única alternativa"
                    saveDebug(string)
                    print("Escolher um lugar para se mexer que não colida com ninguém")
                    with lock:
                        neighbor = mapping.getFreeNeighbor(currentPose,robotInfo["id"])
                        mapping.updateGoals(currentPose,neighbor, neighbor, robotInfo["id"])
                    print("Saindo do meio")
                    distance,angle = AuxiliarFunctions.CalcAngleDistance(currentPose,neighbor)
                    print("Distância:" + str(distance) + "// Angulo:" + str(angle))
                    
                    robot.rotateTo(angle, velocity*0.7)
                    #print("Movimentação 4 do robô: " + str(robotInfo["id"]))
                    robot.moveFoward(distance, angle, velocity*3)
                    currentPose = robot.getAbsolutePosition(False)
                    with lock:
                        mapping.updateGoals(neighbor,neighbor, neighbor, robotInfo["id"])
                        mapping.addStepMap(robotInfo["id"], currentPose)
                    
            

                
    #saveCoord(mapping.visitedList)
    saveMap(mapping.structMap, "map")
    tFinal = datetime.datetime.now()
    deltaT = tFinal- tInit
    print(deltaT)
    #saveEdge(mapping.edgeMap,'arestas')
    print("Terminou")
    
    # # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    sim.simxGetPingTime(clientID)
    
    # # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)

print ('Program started')

#Variáveis do projeto
#radius = 0.15
radius = 0.25
extRadius = radius/math.cos(math.pi/6)

contagem = 0

mapping = Map(extRadius)

sim.simxFinish(-1)

lock = threading.Lock()


threading.Thread(target=Exploring,args=(1,19999,radius)).start()
threading.Thread(target=Exploring,args=(2,19998,radius)).start()
threading.Thread(target=Exploring,args=(3,19997,radius)).start()
#Exploring(4,19995)






