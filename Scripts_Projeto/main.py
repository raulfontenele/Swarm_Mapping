import sim
import time
from Robot import Robot
from Accelerometer import Accelerometer
from Gyroscope import Gyroscope
from Lidar import Lidar
from Node import Node
from Map import Map
from AuxiliarFunctions import AuxiliarFunctions
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
radius = 0.3

sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim

if clientID!=-1:
    print ('Connected to remote API server')
    #Pegar o objeto de Pioneer
    returnCode,khepera=sim.simxGetObjectHandle(clientID,robotBody,sim.simx_opmode_blocking)
        
    #Pegar o objeto da parede de referência
    #returnCode,wallRef=sim.simxGetObjectHandle(clientID,wallReference,sim.simx_opmode_blocking)
    
    
   #Criar objetos do acelerômetro, Lidar e giroscópio
    #accelerometer = Accelerometer("Accelerometer", clientID)
    #gyroscope = Gyroscope("GyroSensor", clientID)
    lidar = Lidar("LaserScannerLaser_2D", clientID)
    
    #Pegar objeto dos motores manipuláveis
    for i in range(2):
        returnCode,handle=sim.simxGetObjectHandle(clientID,motorsName[i],sim.simx_opmode_blocking)
        motorsObject.append(handle)
        
    #Criar o objeto do robô
    robot = Robot(clientID,motorsObject,khepera,lidar)
    
    #Tempo para estabelecer conexões com o servidor
    time.sleep(0.1)
    
    
    #robot.moveFoward(2, 0.02,1)

    #robot.moveFoward(2, 0.02,1)
    '''
    for i in range(15):
        robot.turnRobot()
        time.sleep(1)
        robot.stopRotation()
        time.sleep(0.1)
    '''

    '''  
    #Mover para frente e rotacionar um ângulo no sentido horário e no sentido anti-horario
    
    time.sleep(0.5)
    robot.rotate(-90,0.6)
    time.sleep(0.5)
    robot.rotate(90,0.6)
    '''

    '''
    print(lidar.getBruteDate())
    print(lidar.getBruteDate()[0])
    print(lidar.getBruteDate()[-1])
    '''


    #Teste funcionalidade de scanner de arredor
    #scan = robot.scanAround(60)
    #print(scan)

    #lidar.getBruteDate()
    
    
    '''
    Pegar a posição inicial, verificar os ângulos que podem ser visitados (e adicionalos a lista de
    nós não visitados) e criar o nó inicial.
    Criar o mapa e inicia-lo com o nó inicial
    '''

    initPose = robot.getAbsolutePosition(False)
    neighborhood,angles = robot.scanAround(60)
    
    initNode = Node(initPose,neighborhood,angles)
    mapping = Map(initNode,radius)
    mapping.addVisitedNode([initPose[0],initPose[1]])
    mapping.currentNode = mapping.initNode
    mapping.addNoneVisitedNode(neighborhood)

    '''
        Inicialmente deve-se escanear todos os nós em volta e adicionar aqueles que possuem visinhança
        e que ainda não foram visitados a lista de nós não visitados. Uma vez visitado o nó, ele 
        deverá ser removido da lista de nós não visitados e adicionado a lista de nós visitados.
        
        
        Questão, como fazer para ele caminhar entre os nós??
        Como escolher o mais perto?
        
    '''
    while mapping.noneVisitedList != None :
        '''
            Verificar se todos os vizinhos já foram visitados. Caso algum ainda não tenha sido, 
            visitar ele.
        '''
        
        if len(mapping.currentNode.neighborhoodCoord) > 0:
            if mapping.checkVisited(mapping.currentNode.neighborhoodCoord[0]) == False:
                print("Nó não visitado")
                print("Coordenadas destino: " + str(mapping.currentNode.neighborhoodCoord[0]) )
                robot.rotateTo(mapping.currentNode.neighborhoodAngle[0][2], 1.5*velocity)
                robot.moveFoward(2*radius, 0, velocity*3)
                
                mapping.visitedNode(mapping.currentNode.neighborhoodCoord[0])
                
                '''
                Uma vez que se chegou no nó não visitado, deve-se escanear em volta, e adicionar os visinhos, não visitados,
                a lista de nós não visitados, atualizar o nó atual do mapa e dizer que o nó atual é filho do nó anterior.
                Também deve-se adiconar o nó atual a lista de visitados
                '''
                currentPose = robot.getAbsolutePosition(False)
                neighborhood,angles = robot.scanAround(60)
                mapping.addNoneVisitedNode(neighborhood)

                currentNode = Node(currentPose,neighborhood,angles,mapping.currentNode,mapping.currentNode.neighborhoodAngle[0][2])

                #Excluir da lista de visinhos possíveis e incluir na lista de nós visitados
                #mapping.currentNode.deleteNeighborhoodCoord(mapping.currentNode.neighborhoodCoord[0])
                #mapping.currentNode.deleteNeighborhoodAngle(mapping.currentNode.neighborhoodAngle[0])
                mapping.currentNode.neighborhoodCoord.remove(mapping.currentNode.neighborhoodCoord[0])
                mapping.currentNode.neighborhoodAngle.remove(mapping.currentNode.neighborhoodAngle[0])
                
                mapping.currentNode = currentNode
                
                
                
            else:
                print("nó visitado")
                print("Coordenadas visitadas: " + str(mapping.currentNode.neighborhoodCoord[0]) )
                #Excluir da lista de visinhos possíveis
                mapping.currentNode.neighborhoodCoord.remove(mapping.currentNode.neighborhoodCoord[0])
                mapping.currentNode.neighborhoodAngle.remove(mapping.currentNode.neighborhoodAngle[0])

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
                robot.moveFoward(2*radius, 0, velocity*3)
                mapping.currentNode = mapping.currentNode.parent
            else:
                print("Chegou no nó inicial")
                break
    

    arquivo = open("Coordenadas.txt",'a')
    arquivo.writelines(str(mapping.visitedList))
    arquivo.close()

    
            
                
        
        

           

    
    
    
    '''
    while(mapping.currentNode.angles != [False,False,False,False,False,False]):
        for index in range(len(mapping.currentNode.angles)):
            if mapping.currentNode.angles[index] == True:
                #Rotacionar e ir até o nó
                robot.rotate(60,0.6)
                robot.moveFoward(0.3,0.02,1)
                
                #Verificar se o n todas as possibilidades e adicionar o nó
                currentPose = robot.getAbsolutePosition(False)
                
                mapping.addVisitedNode(currentPose)
                
                scan = robot.scanAround(60)
                
            
                newNode = Node(currentPose,scan)
                
                
                
                mapping.currentNode.angles[index] = False
                
                robot.moveFoward(-0.3,0.02,1)
    
    '''
    

        

    
    
    '''
    #Teste de rotação do motores
    i = 0
    robot.turnRobot()
    while True:
        print(robot.getAbsoluteOrientation(False))
        i+=1
        time.sleep(0.1)
        if i > 300:
            break
    robot.stopRotation()
    '''

    
    # Now send some data to CoppeliaSim in a non-blocking fashion:
    sim.simxAddStatusbarMessage(clientID,'Hello CoppeliaSim!',sim.simx_opmode_oneshot)

    # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    sim.simxGetPingTime(clientID)

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')
