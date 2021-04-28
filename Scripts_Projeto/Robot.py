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
from Save import saveDebug

class Robot:
    def __init__(self,idClient,robotId,motorsId,robotObject,lidar):
        self.idClient = idClient
        self.motorsId = motorsId
        self.robot = robotObject
        #self.accelerometer = accelerometer
        self.lidar = lidar
        #self.gyroscope = gyroscope
        #self.ownRadius = 0.07
        #self.radius = 0.15
        self.ownRadius = 0.15
        self.radius = 0.5
        self.extRadius = self.radius/math.cos(math.pi/6)
        self.id = robotId
        
        #Inicialização da função de posição
        self.getAbsolutePosition(True)
        self.getAbsoluteOrientation(True)
        
    
    def rotate(self,angle,velocity):
        if angle != 0:
            #Abordagem utilizando a orientação absoluta
            initOrientation = self.getAbsoluteOrientation(False)
            
            # Realizar o controle do angulo de rotação
            if angle > 0:
                sim.simxSetJointTargetVelocity(self.idClient,self.motorsId[0],velocity,sim.simx_opmode_oneshot)
                sim.simxSetJointTargetVelocity(self.idClient,self.motorsId[1],-velocity,sim.simx_opmode_oneshot)
            else:
                sim.simxSetJointTargetVelocity(self.idClient,self.motorsId[0],-velocity,sim.simx_opmode_oneshot)
                sim.simxSetJointTargetVelocity(self.idClient,self.motorsId[1],velocity,sim.simx_opmode_oneshot)
            
    
            #print("orientação inicial giroscópio")
            #gir = self.gyroscope.getGyroscoperReadVrep()
         
            #print("Orientação inicial")
            #print(initOrientation)
    
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
        #self.lidar.getBruteDate()
        #self.lidar.getPointRead()
        
        
    def getAbsoluteOrientation(self,flag):
        if flag == True:
            returnCode,eulerAngles = sim.simxGetObjectOrientation(self.idClient,self.robot,-1,sim.simx_opmode_streaming)
        else:
            returnCode,eulerAngles = sim.simxGetObjectOrientation(self.idClient,self.robot,-1,sim.simx_opmode_buffer)
        
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
        for i in range(len(self.motorsId)):
            sim.simxSetJointTargetVelocity(self.idClient,self.motorsId[i],0,sim.simx_opmode_oneshot)
            
    def moveFoward(self,distance,angle,velocity):
        #threshold = 2*self.radius + 1.4*self.ownRadius
 
        if distance > 0:
            #Ligar os motores pra frente
            self.turnOnRobot(velocity,velocity,'front')
        else:
            #Ligar os motores pra trás
            self.turnOnRobot(velocity,velocity,'back')


        #Abordagem utilizando a posição absoluta
        initPosition = self.getAbsolutePosition(False)

        distanceAbs = 0
        
        P = 0.017

        while(distanceAbs <= abs(distance)):
                     
            position = self.getAbsolutePosition(False)
            distanceAbs = math.sqrt((initPosition[0]-position[0])**2 + (initPosition[1]-position[1])**2)
            
            #Aplicar um controlador P
            error = AuxiliarFunctions.diffAngleThreshold(angle,self.getAbsoluteOrientation(False)[2])
            #print(error)
            rightVelocity = velocity*(1 - error*P)
            leftVelocity = velocity*(1 + error*P)
            self.turnOnRobot(leftVelocity,rightVelocity,'front')
            #if self.checkDistance(self.ownRadius) ==  False:
                #print("Parou por ir para frente por estar muito próximo a parede")
                #break

        self.stopRotation()
        return distanceAbs

        
    def getAbsolutePosition(self,flag):
        if flag == True:
            returnCode,position = sim.simxGetObjectPosition(self.idClient,self.robot,-1,sim.simx_opmode_streaming )
        else:
            returnCode,position = sim.simxGetObjectPosition(self.idClient,self.robot,-1,sim.simx_opmode_buffer )
        return position
    
    def scanAround(self,velocity,mapping):
        nodes = []
        angles = []
        #distances = []
        #error = 0
        angList = [0,60,120,180,240,300]
        
        initCoord = self.getAbsolutePosition(False)
        #angList = []
        '''
        for angle in ang:
            coord = AuxiliarFunctions.projectCoord(angle, initCoord, 2*self.radius)
            flag,node = mapping.checkVisited(coord)
            if flag == False:
                angList.append(angle)      
            else:
                nodes.append(node)
                angles.append(angle)
        #print("Angulos visitados: " + str(angles))
        '''
        for i in range(len(angList)):
            
            self.rotateTo(angList[i], velocity)
                  
            coord = AuxiliarFunctions.projectCoord(angList[i], initCoord, 2*self.radius)
            #if self.checkDistance(threshold) ==  True or mapping.checkGoalAnother(coord, 'next') == True:
            if self.lidar.getDetectedState(True) ==  False or mapping.checkGoalAnother(coord, 'next', self.id) == True:
                nodes.append(coord)
                angles.append(angList[i])

            time.sleep(0.15)
            
        logNeighbor(nodes,angles,initCoord,self.id)
        if len(nodes) == 0:
            saveDebug("Deu problema porque o nó " + str(initCoord) + " não tem visinho")
            print("Nó sem visinhos")

        return nodes,angles

        
    def turnOnRobot(self,velocityR,velocityL,rotationSense):
        if rotationSense == "hor":
            sim.simxSetJointTargetVelocity(self.idClient,self.motorsId[0],velocityL,sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(self.idClient,self.motorsId[1],-velocityR,sim.simx_opmode_oneshot)
        elif rotationSense == "antihor":
            sim.simxSetJointTargetVelocity(self.idClient,self.motorsId[0],-velocityL,sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(self.idClient,self.motorsId[1],velocityR,sim.simx_opmode_oneshot)
        elif rotationSense == "front":
            sim.simxSetJointTargetVelocity(self.idClient,self.motorsId[0],velocityL,sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(self.idClient,self.motorsId[1],velocityR,sim.simx_opmode_oneshot)
        else:
            sim.simxSetJointTargetVelocity(self.idClient,self.motorsId[0],-velocityL,sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(self.idClient,self.motorsId[1],-velocityR,sim.simx_opmode_oneshot)
        
        
    def rotateTo(self,angle,velocity):
        #Verificar pra qual lado é mais curto girar
        currentOrientation = self.getAbsoluteOrientation(False)
        index =2
        
        diffHor = AuxiliarFunctions.diffAngles(angle,currentOrientation[index] , -1)
        diffAntiHor = AuxiliarFunctions.diffAngles(angle,currentOrientation[index] , 1)

        
        if diffHor < diffAntiHor:
            self.turnOnRobot(velocity,velocity,'hor')
            sign = 1
        else:
            self.turnOnRobot(velocity,velocity,'antihor')
            sign = -1

        while(True):
            currentOrientation = self.getAbsoluteOrientation(False)
            diff = AuxiliarFunctions.diffAngles(angle,currentOrientation[index] , sign)
            
            if diff <= 0.4:
                self.stopRotation()
                '''
                print("Inicial::" + str(angle))
                print("Final::" + str(currentOrientation[index]))
                print("Sinal::" + str(sign))
                print("Diferença::" + str(diff))
                '''
                break

            
            
    def checkDistance(self,thresholdMax,thresholdMin = 0.1):
        '''
        No momento de scanear, verificar se a leitura que deu errado num foi por causa do outro robô
        
        Executar apenas uma vez a verificação, pra ver o que acontece
        '''

        
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
        

    


        
        
        
        
        
        