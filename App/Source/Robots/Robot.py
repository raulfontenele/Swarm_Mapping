# -*- coding: utf-8 -*-
"""
Created on Sat Feb  6 18:24:18 2021

@author: raulf
"""
import sim
import time
import math
from AuxiliarFunctions import AuxiliarFunctions
from debug import logNeighbor,logLidar
from Save import saveDebug, saveCoordinate, saveOrientation
from datetime import datetime

from Enums import *
#import Robot.
from Robots.IRobot import IRobot

class Robot(IRobot):
    def __init__(self,communicationApi,robotInfos,lidar,radiusZone):
        
        self.comm = communicationApi
        self.robotInfos = robotInfos

        self.lidar = lidar
        self.ownRadius = 0.15
        self.radius = radiusZone
        self.extRadius = self.radius/math.cos(math.pi/6)
        #self.id = robotId
        
        #Inicialização da função de posição
        self.getAbsolutePosition(True)
        self.getAbsoluteOrientation(True)
        
    
    def rotate(self,angle,velocity):
        if angle != 0:
            #Abordagem utilizando a orientação absoluta
            initOrientation = self.getAbsoluteOrientation(False)
            
            # Realizar o controle do angulo de rotação
            if angle > 0:
                sim.simxSetJointTargetVelocity(self.comm.clientId,self.robotInfos["leftMotor"],velocity,sim.simx_opmode_oneshot)
                sim.simxSetJointTargetVelocity(self.comm.clientId,self.robotInfos["rightMotor"],-velocity,sim.simx_opmode_oneshot)
            else:
                sim.simxSetJointTargetVelocity(self.comm.clientId,self.robotInfos["leftMotor"],-velocity,sim.simx_opmode_oneshot)
                sim.simxSetJointTargetVelocity(self.comm.clientId,self.robotInfos["rightMotor"],velocity,sim.simx_opmode_oneshot)
            

    
            sign = 1 if angle > 0  else -1
            # Rotação no eixo Z
            index = 2
            while(True):
                currentOrientation = self.getAbsoluteOrientation(False)
                diff = AuxiliarFunctions.diffAngles(initOrientation[index],currentOrientation[index] , sign)
                if diff >= abs(angle) and abs(initOrientation[index] - currentOrientation[index]) > 1:
                    self.stopRotation()
                    '''
                    print("Inicial::" + str(initOrientation[index]))
                    print("Final::" + str(currentOrientation[index]))
                    print("Sinal::" + str(sign))
                    print("Diferença::" + str(diff))
                    '''
                    break
                    
    
            return AuxiliarFunctions.diffAngles(initOrientation[index], self.getAbsoluteOrientation(False)[index], sign) - abs(angle)
        
        else:
            return 0
        
        
    def getAbsoluteOrientation(self,flag):
        if flag == True:
            returnCode,eulerAngles = sim.simxGetObjectOrientation(self.comm.clientId,self.robotInfos["robot"],-1,sim.simx_opmode_streaming)
        else:
            returnCode,eulerAngles = sim.simxGetObjectOrientation(self.comm.clientId,self.robotInfos["robot"],-1,sim.simx_opmode_buffer)
        
        #Com a conversão dos angulos de euler
        '''
            Os valores dos ângulos de euler variam de 0 a -2*pi entre os ângulos de 90º e -90º nos quadrantes 1 e 4
            enquanto os valores 0 a 2*pi entre os ângulos de 90º e -90º nos quadrantes 2 e 3.
            A conversão consiste em transformar em valores de 0 a 360º com 90º sendo o zero e crescendo no sentido horário
            
        '''
        
        for i in range(len(eulerAngles)):
            if eulerAngles[i] < 0:
                eulerAngles[i] = abs(eulerAngles[i])*180/math.pi
            else:
                eulerAngles[i] = (math.pi + abs(math.pi - eulerAngles[i]))*180/math.pi
            if eulerAngles[i] >= 360:
                eulerAngles[i] -= 360
        
        #print("angulos de euler graus:")
        return eulerAngles
    
    def stopRotation(self):
        self.comm.Pause(True)
        sim.simxSetJointTargetVelocity(self.comm.clientId,self.robotInfos["leftMotor"],0,sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(self.comm.clientId,self.robotInfos["rightMotor"],0,sim.simx_opmode_oneshot)
        self.comm.Pause(False)
            
            
    def moveFoward(self,distance,angle,velocity):
        #threshold = 2*self.radius + 1.4*self.ownRadius
 
        if distance > 0:
            #Ligar os motores pra frente
            self.turnOnRobot(velocity,velocity,Moviment.Foward)
        else:
            #Ligar os motores pra trás
            self.turnOnRobot(velocity,velocity,Moviment.Back)


        #Abordagem utilizando a posição absoluta
        initPosition = self.getAbsolutePosition(False)

        distanceAbs = 0
        
        #Ku = 0.545
        #Tu = 0.0063851
        
        #Kp = 0.6*Ku
        #Ki = 2*Kp/Tu
        #Kd = Kp*Tu/8
        
        #Kp = 0.28
        #Ki = 0.008
        #Kd = 0.0009
        
        Kp = 0
        Ki = 0
        Kd = 0
        
        
        initTime = datetime.now()
        lastTime = datetime.now()
        
        accError = 0
        lastError = AuxiliarFunctions.diffAngleThreshold(angle,self.getAbsoluteOrientation(False)[2])
        time.sleep(0.001)
        #cicle = 0
        #flag = False
        while(distanceAbs <= abs(distance)-0.005):
                     
            position = self.getAbsolutePosition(False)
            distanceAbs = math.sqrt((initPosition[0]-position[0])**2 + (initPosition[1]-position[1])**2)
            
            
            #Aplicar um controlador P
            error = AuxiliarFunctions.diffAngleThreshold(angle,self.getAbsoluteOrientation(False)[2])
            #if abs(error) < 0.01 and flag == False and cicle > 0:
                #print("//==========================================================================//")
                #print(cicle)
                #print("//==========================================================================//")
                #flag = True
            dt = float((datetime.now() - lastTime).microseconds/1000000 - 0.0001)
            
            t = float((datetime.now() - initTime).total_seconds())
            saveCoordinate(self.getAbsoluteOrientation(False)[2], t, position)
            #print("Tempo")
            #print(dt)
            #print(datetime.now().microsecond)
            accError += error*dt
            #print("Acumulado")
            #print(accError)
            rateError = (error - lastError)/dt

            correctError = Kp*error + Ki*accError + Kd*rateError
            

            rightVelocity = velocity*(1 - correctError)
            leftVelocity = velocity*(1 + correctError)
            self.turnOnRobot(leftVelocity,rightVelocity,Moviment.Foward)
            #if self.checkDistance(self.ownRadius) ==  False:
                #print("Parou por ir para frente por estar muito próximo a parede")
                #break
            lastTime = datetime.now()
            lastError = error
            #cicle += 1
            time.sleep(0.001)
        self.stopRotation()
        return distanceAbs

        
    def getAbsolutePosition(self,flag):
        if flag == True:
            returnCode,position = sim.simxGetObjectPosition(self.comm.clientId,self.robotInfos["robot"],-1,sim.simx_opmode_streaming )
        else:
            returnCode,position = sim.simxGetObjectPosition(self.comm.clientId,self.robotInfos["robot"],-1,sim.simx_opmode_buffer )
        return position
    
    def scanAround(self,velocity,mapping,initCoord):
        nodes = []
        angles = []
        #distances = []
        #error = 0
        angList = [0,60,120,180,240,300]
        #print("Lista de angulos:{}".format(angList))
        '''
            Se meu nó atual já for o vizinho de alguém, adicionar o nó pai como me vizinho
        '''
        neighborhood = mapping.CheckNeighborSome(initCoord)
        #print("vizinhos nessa porrinha{}".format(neighborhood))

        for neighbor in neighborhood:
            #Descobrir qual dos angulos que eu deveria ir e adicionar o nó conhecido como vizinho
            opositeAngle = AuxiliarFunctions.oppositeAngle(neighbor["angle"])
            #print("Angulo oposto:{}".format(opositeAngle))
            j = [abs(i-opositeAngle) for i in angList]
            realAngle = angList[j.index(min(j))]
            #print("Angulo real:{}".format(realAngle))
            angList.remove(realAngle)
            nodes.append(neighbor["nodeCoord"])
            angles.append(realAngle)
 
        for i in range(len(angList)):
            
            self.rotateTo(angList[i], velocity)
                  
            coord = AuxiliarFunctions.projectCoord(angList[i], initCoord, 2*self.radius)
            # Se for coordenada de outro, adicionar a coordenada 
            #or mapping.checkGoalAnother(coord, 'next', self.robotInfos["ID"]) == True
            if self.lidar.getDetectedState(True) == False :
                nodes.append(coord)
                angles.append(angList[i])

            time.sleep(0.1)
            
        self.rotateTo(0, velocity)
        
        logNeighbor(nodes,angles,initCoord,self.robotInfos["ID"])
        if len(nodes) == 0:
            saveDebug("Deu problema porque o nó " + str(initCoord) + " não tem visinho")
            print("Nó sem visinhos")
        #print("Nodes : {}".format(nodes))
        #print("Angles : {}".format(angles))
        #nodesS = [x for _, x in sorted(zip(angles, nodes))]
        #anglesS = sorted(angles)   
        #listSorted = sorted(coordUnique, key=lambda x: x["number"])
        #return nodes,angles

        #print("Nodes  sorted: {}".format(nodesS))
        #print("Angles sorted : {}".format(anglesS))

        return nodes, angles

        
    def turnOnRobot(self,velocityR,velocityL,rotationSense):
        self.comm.Pause(True)
        if rotationSense == Moviment.Hor:
            sim.simxSetJointTargetVelocity(self.comm.clientId,self.robotInfos["leftMotor"],velocityL,sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(self.comm.clientId,self.robotInfos["rightMotor"],-velocityR,sim.simx_opmode_oneshot)
        elif rotationSense == Moviment.Antihor:
            sim.simxSetJointTargetVelocity(self.comm.clientId,self.robotInfos["leftMotor"],-velocityL,sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(self.comm.clientId,self.robotInfos["rightMotor"],velocityR,sim.simx_opmode_oneshot)
        elif rotationSense == Moviment.Foward:
            sim.simxSetJointTargetVelocity(self.comm.clientId,self.robotInfos["leftMotor"],velocityL,sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(self.comm.clientId,self.robotInfos["rightMotor"],velocityR,sim.simx_opmode_oneshot)
        else:
            sim.simxSetJointTargetVelocity(self.comm.clientId,self.robotInfos["leftMotor"],-velocityL,sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(self.comm.clientId,self.robotInfos["rightMotor"],-velocityR,sim.simx_opmode_oneshot)  
        self.comm.Pause(False)

        
        
        
    def rotateTo(self,angle,velocity):
        #Verificar pra qual lado é mais curto girar
        currentOrientation = self.getAbsoluteOrientation(False)
        index =2
        
        diffHor = AuxiliarFunctions.diffAngles(angle,currentOrientation[index] , -1)
        diffAntiHor = AuxiliarFunctions.diffAngles(angle,currentOrientation[index] , 1)

        
        if diffHor < diffAntiHor:
            self.turnOnRobot(velocity,velocity,Moviment.Hor)
            sign = 1
        else:
            self.turnOnRobot(velocity,velocity,Moviment.Antihor)
            sign = -1

        while(True):
            currentOrientation = self.getAbsoluteOrientation(False)
            diff = AuxiliarFunctions.diffAngles(angle,currentOrientation[index] , sign)
            if diff <= 0.4:
                self.stopRotation()
                break

            

    '''       
    def checkDistance(self,thresholdMax,thresholdMin = 0.1):
        
        #No momento de scanear, verificar se a leitura que deu errado num foi por causa do outro robô
        
        #Executar apenas uma vez a verificação, pra ver o que acontece
        

        
        cont = 0

        #for i in range(6):
        bruteLidar = self.lidar.getBruteDate()
        #distance = self.lidar.getPointRead()[2]

        #Projetar quando da erro
        #Percorrer metades de todos os pontos do lidar
        flagProximity = False
        angLimit = (math.pi/2 - math.atan((self.ownRadius + 0.03)/(2*self.radius + self.ownRadius + 0.03)))*180/math.pi
        ang = 0
        interator = 1
        for index in range(0,len(bruteLidar),2):
            #dist = math.sqrt(bruteLidar[index][1]**2 + bruteLidar[index][2]**2)
            if ang < angLimit and abs(bruteLidar[index][2])<self.ownRadius + 0.02 :
                flagProximity = True
                break
            elif ang>=angLimit and abs(bruteLidar[index][1]) <= thresholdMax:
                flagProximity = True
                break
            if ang == 90:
                interator*=-1
            ang += interator
            
        if flagProximity == True:
            cont+=1

            time.sleep(0.1)
            
        if cont<=0:
            return True
        else:
            #logLidar(lidarCenter,lidarLeft,lidarRight)
            return False
    '''
        

    


        
        
        
        
        
        