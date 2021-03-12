import sim
import time
from Robot import Robot
from Lidar import Lidar
from Node import Node
from Map import Map
from AuxiliarFunctions import AuxiliarFunctions
from Save import saveCoord,saveEdge

print ('Program started')

#Variáveis do projeto
#motorsName = ['Pioneer_p3dx_leftMotor','Pioneer_p3dx_rightMotor']
#robotName = 'Pioneer_p3dx'
motorsName = ['K3_leftWheelMotor','K3_rightWheelMotor']
robotName = 'K3_robot'
robotBody = 'K3_bodyPart1'
motorsObject = []
wallReference = 'ConcretBlock'

velocity = 0.6
radius = 0.25

contagem = 0

sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim

if clientID!=-1:
    print ('Connected to remote API server')
    
    #Pegar o objeto de Pioneer
    returnCode,khepera=sim.simxGetObjectHandle(clientID,robotBody,sim.simx_opmode_blocking)

   #Criar objetos do acelerômetro, Lidar e giroscópio
    lidar = Lidar("LaserScannerLaser_2D", clientID,28,2)
    
    #Pegar objeto dos motores manipuláveis
    for i in range(2):
        returnCode,handle=sim.simxGetObjectHandle(clientID,motorsName[i],sim.simx_opmode_blocking)
        motorsObject.append(handle)
        
    #Criar o objeto do robô
    robot = Robot(clientID,motorsObject,khepera,lidar)
    
    #Tempo para estabelecer conexões com o servidor
    time.sleep(0.1)
    
    
    '''
    Inicialmente deve-se pegar a posição inicial e realizar o processo de scanner, afim de obter os nós visinhos
    e criar um nó contendo as informações obtidas. Em seguida deve-se criar o mapa, adicionar o nó inicial a ele,
    bem como definir o nó atual como o nó inicial. Por fim, deve-se adicionar o nó atual a lista de nós visitados
    e os nós visinhos a lista de nós não visitados.
    '''

    initPose = robot.getAbsolutePosition(False)
    neighborhood,angles = robot.scanAround(60)
    
    initNode = Node(initPose)
    initNode.addNeighborhood(neighborhood, angles)
    
    mapping = Map(initNode,radius)
    mapping.currentNode = mapping.initNode
    mapping.addVisitedNode(initPose)
    mapping.addNoneVisitedNode(neighborhood)

    '''
        Inicialmente deve-se escanear todos os nós em volta e adicionar aqueles que possuem visinhança
        e que ainda não foram visitados a lista de nós não visitados. Uma vez visitado o nó, ele 
        deverá ser removido da lista de nós não visitados e adicionado a lista de nós visitados.
        
        
        Questão, como fazer para ele caminhar entre os nós??
        Como escolher o mais perto?
        Como fazer ele evitar o caminho mais longo, quando possível?
        A cada passo, na hora de voltar, verificar se pode pular por visinhos?
        
    '''
    while mapping.noneVisitedList != None :
        '''
            Verificar se todos os vizinhos já foram visitados. Caso algum ainda não tenha sido, 
            visitar ele.
        '''
        
        if mapping.currentNode.checkNeighborhood() == True:
            neighborCoord,neighborAngle = mapping.currentNode.getNeighbor()
            print("Index atual: " + str(mapping.currentNode.visitedIndex) + " -- De: " + str(len(mapping.currentNode.neighborhoodCoord)))
            if mapping.checkVisited(neighborCoord) == False:
                
                print("Nó não visitado")
                print("Coordenadas destino: " + str(neighborCoord) )
                robot.rotateTo(neighborAngle[2], velocity)
                distParent,flag = robot.moveFoward(2*radius, 0, velocity*3)
                print("Distancia deslocada: " + str(distParent) )
                
                if flag == True:
                    '''
                    Uma vez que se chegou no nó não visitado, deve-se escanear em volta, e adicionar os visinhos, não visitados,
                    a lista de nós não visitados, atualizar o nó atual do mapa e dizer que o nó atual é filho do nó anterior.
                    Também deve-se adiconar o nó atual a lista de visitados, assim como dizer que o atual nó, visinho do pai, foi visitado
                    '''
                    
                    #mapping.currentNode.confirmVisitedNeighbor()
                    currentPose = robot.getAbsolutePosition(False)
                    mapping.visitedNode(currentPose)
                    neighborhood,angles = robot.scanAround(60)
                    mapping.addNoneVisitedNode(neighborhood)
    
                    currentNode = Node(currentPose,mapping.currentNode,neighborAngle[2],distParent)
                    currentNode.addNeighborhood(neighborhood, angles)
                    
                    mapping.currentNode = currentNode
                else:
                    print("Falha ao tentar ir para nó visinho")
                    contagem +=1
                
            else:
                print("nó visitado")
                print("Coordenadas visitadas: " + str(neighborCoord) )

        else:
            '''
                Caso não haja mais nós visinhos a serem visitados, deverá retornar ao nó pai e 
                repetir o processo até que não hajam mais nós a serem visitados.
                Para retornar ao pai, o robô derá pegar o ângulo de entrada para o nó atual, calcular
                o simétrico dele e andar de novo o caminho inverso que realizou.
                Também deverá ser atualizado o nó atual do mapa
            '''
            print("Acabaram os nós visinhos. Voltando ao pai")
            if mapping.currentNode.parent != None:
                opossitAngle = AuxiliarFunctions.oppositeAngle(mapping.currentNode.parentAngle)
                robot.rotateTo(opossitAngle, velocity)
                robot.moveFoward(mapping.currentNode.parentDist, 0, velocity*3)
                print("Distancia ao nó pai: " + str(mapping.currentNode.parentDist))
                mapping.currentNode = mapping.currentNode.parent
                
            else:
                print("Chegou no nó inicial")
                break
    
    '''
    arquivo = open("Coordenadas.txt",'a')
    arquivo.writelines(str(mapping.visitedList))
    arquivo.close()
    
    arquivo = open("arestas.txt",'a')
    arquivo.writelines(str(mapping.matrixMap))
    arquivo.close()
    '''

    saveCoord(mapping.visitedList)
    saveEdge(mapping.matrixMap)
    

    print("Quantidade de vezes na excessão: " + str(contagem))
    # Now send some data to CoppeliaSim in a non-blocking fashion:
    sim.simxAddStatusbarMessage(clientID,'Hello CoppeliaSim!',sim.simx_opmode_oneshot)

    # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    sim.simxGetPingTime(clientID)

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')


'''
    Mudança a se fazer:
        Quando for detectado que um nó visinho já foi visitado, dar um jeito de gravar que o nó visinho
        também é acessível por esse nó. Isso irá criar um mapa que mostra todas as ligações possíveis
'''