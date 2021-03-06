# -*- coding: utf-8 -*-
"""
Created on Sat Feb  6 18:24:18 2021

@author: raulf
"""
import sim
import time
import math
from AuxiliarFunctions import AuxiliarFunctions

class Robot:
    def __init__(self,idClient,motorsId,robotObject,lidar):
        self.idClient = idClient
        self.motorsId = motorsId
        self.robot = robotObject
        #self.accelerometer = accelerometer
        self.lidar = lidar
        #self.gyroscope = gyroscope
        
        self.radius = 0.3
        
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
        
        '''
        #Sem a conversão de euler
        for i in range(len(eulerAngles)):
            eulerAngles[i] = eulerAngles[i]*180/math.pi
        '''
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
            
    def moveFoward(self,distance,interval,velocity):
        if distance > 0:
            #Ligar os motores pra frente
            sim.simxSetJointTargetVelocity(self.idClient,self.motorsId[0],velocity,sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(self.idClient,self.motorsId[1],velocity,sim.simx_opmode_oneshot)
        else:
            #Ligar os motores pra trás
            sim.simxSetJointTargetVelocity(self.idClient,self.motorsId[0],-velocity,sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(self.idClient,self.motorsId[1],-velocity,sim.simx_opmode_oneshot)
        '''    
        print("Acelerção VREP inicial")
        acceleration = self.accelerometer.getAccelerometerReadVrep()
        print(acceleration)
        '''
          
        #Abordagem utilizando a posição absoluta
        initPosition = self.getAbsolutePosition(False)
        #print("Posição inicial global")
        #print(initPosition)

        distanceAbs = 0

        while(distanceAbs <= abs(distance)):
                     
            position = self.getAbsolutePosition(False)
            distanceAbs = math.sqrt((initPosition[0]-position[0])**2 + (initPosition[1]-position[1])**2)
            if self.lidar.getPointRead()[2] <= 0.13:
                print("Parou por ir para frente por estar muito próximo a parede")
                print(self.lidar.getPointRead()[2])
                break
            #self.lidar.getBruteDate()

        self.stopRotation()
        
    def getAbsolutePosition(self,flag):
        if flag == True:
            returnCode,position = sim.simxGetObjectPosition(self.idClient,self.robot,-1,sim.simx_opmode_streaming )
        else:
            returnCode,position = sim.simxGetObjectPosition(self.idClient,self.robot,-1,sim.simx_opmode_buffer )
        return position
    
    def scanAround(self,angleBase):
        nodes = []
        angles = []
        error = 0
        ang = [0,60,120,180,240,300]

            
        initCoord = self.getAbsolutePosition(False)
        print("//==============================================//")
        print("Coordenadas do nó atual: " + str(initCoord))

        
        for i in range(6):
            #error = self.rotate(angleBase-error,0.4)
            self.rotateTo(ang[i], 0.4)
            #print(error)
            bruteLidar = self.lidar.getBruteDate()
            if self.lidar.getPointRead()[2] >= 3.5*self.radius and bruteLidar[0][1]>= 3.5*self.radius and bruteLidar[-1][1]>= 3.5*self.radius:
                
                orientation = self.getAbsoluteOrientation(False)
                
                
                coord = AuxiliarFunctions.projectCoord(orientation, initCoord, 2*self.radius)
                print("Distancia lida:" + str(self.lidar.getPointRead()[2]))
                print("Orientação atual:" + str(orientation) )
                
                nodes.append(coord)
                angles.append(orientation)
                
                #Mostrar a coordenada atual e a projeção dos visinhos
                print("Coordenadas do nó visinho: " + str(coord))

            time.sleep(0.5)
        print("//==============================================//")
        return nodes,angles

        
    def turnRobot(self):
        sim.simxSetJointTargetVelocity(self.idClient,self.motorsId[0],0.5,sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(self.idClient,self.motorsId[1],-0.5,sim.simx_opmode_oneshot)
        
        
    def rotateTo(self,angle,velocity):
        #Ligar os motores
        sim.simxSetJointTargetVelocity(self.idClient,self.motorsId[0],velocity,sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(self.idClient,self.motorsId[1],-velocity,sim.simx_opmode_oneshot)
        
        #Abordagem utilizando a orientação absoluta
        #initOrientation = self.getAbsoluteOrientation(False)
        index = 2
        sign = 1
        while(True):
            currentOrientation = self.getAbsoluteOrientation(False)
            diff = AuxiliarFunctions.diffAngles(angle,currentOrientation[index] , sign)
            
            if diff <= 1:
                self.stopRotation()
                '''
                print("Inicial::" + str(angle))
                print("Final::" + str(currentOrientation[index]))
                print("Sinal::" + str(sign))
                print("Diferença::" + str(diff))
                '''
                
                break
    '''
    def setOrientation(self,angle):
        returnCode = sim.simxSetObjectOrientation(self.idClient,self.robot,-1,[0,0,angle],sim.simx_opmode_oneshot)
        print(returnCode)
        	
    def setPosition(self,position):
        returnCode = sim.simxSetObjectPosition(self.idClient,self.robot,-1,position,sim.simx_opmode_oneshot)
        print(returnCode)	
    '''
    


        
        
        
        
        
        