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


def Exploring(robotFile,comPort):
    #Definição de variáveis
    motorsObject = []
    velocity = 0.7
    #velocity = 0.4
    radius = 0.25
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
    #lidar = 10
    
    #print("Passou 1")
     
    #Pegar objeto dos motores manipuláveis
    for i in range(2):
        returnCode,handle=sim.simxGetObjectHandle(clientID,robotInfo["motorsName"][i],sim.simx_opmode_blocking)
        motorsObject.append(handle)
    
    print("Passou 2")
        
    #Criar o objeto do robô
    robot = Robot(clientID,robotInfo["id"],motorsObject,khepera,lidar)
    
    print("Passou 3")
    
    #Tempo para estabelecer conexões com o servidor
    time.sleep(0.2)
    print("Passou 4")
    #robot.moveFoward(2, 0, velocity*3)
    tfinal = datetime.datetime.now()
    
    #print(tfinal - tInit)
    for i in range(500):
        if i == 0:
            lidar.getDetectedState(True)
        else:
            lidar.getDetectedState(True)
        time.sleep(0.1)
        
    #lidar.teste()
    
    # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    sim.simxGetPingTime(clientID)
    
    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)

print ('Program started')

#Variáveis do projeto
#motorsName = ['Pioneer_p3dx_leftMotor','Pioneer_p3dx_rightMotor']
#robotName = 'Pioneer_p3dx'
'''
motorsName = ['K3_leftWheelMotor','K3_rightWheelMotor']
robotName = 'K3_robot'
robotBody = 'K3_bodyPart1'
motorsObject = []
wallReference = 'ConcretBlock'
'''

radius = 0.25
extRadius = radius/math.cos(math.pi/6)

contagem = 0

mapping = Map(extRadius)


robot1str = "robot1"
robot2str = "robot2"

threading.Thread(target=Exploring,args=(1,19999)).start()
#threading.Thread(target=Exploring,args=(2,19998)).start()
#Exploring(1,19999)