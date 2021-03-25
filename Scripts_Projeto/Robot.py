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

class Robot:
    def __init__(self,idClient,robotId,motorsId,robotObject,lidar):
        self.idClient = idClient
        self.motorsId = motorsId
        self.robot = robotObject
        #self.accelerometer = accelerometer
        self.lidar = lidar
        #self.gyroscope = gyroscope
        self.ownRadius = 0.15
        self.radius = 0.25
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
        #threshold = 2*self.radius + 1.4*self.ownRadius
 
        if distance > 0:
            #Ligar os motores pra frente
            self.turnOnRobot(velocity,'front')
        else:
            #Ligar os motores pra trás
            self.turnOnRobot(velocity,'back')


        #Abordagem utilizando a posição absoluta
        initPosition = self.getAbsolutePosition(False)

        distanceAbs = 0

        while(distanceAbs <= abs(distance)):
                     
            position = self.getAbsolutePosition(False)
            distanceAbs = math.sqrt((initPosition[0]-position[0])**2 + (initPosition[1]-position[1])**2)
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
        distances = []
        #error = 0
        ang = [0,60,120,180,240,300]
        
        initCoord = self.getAbsolutePosition(False)
        angList = []
        
        for angle in ang:
            coord = AuxiliarFunctions.projectCoord(angle, initCoord, 2*self.radius)
            flag,node = mapping.checkVisited(coord,False)
            if flag == False:
                angList.append(angle)
                
            else:
                nodes.append(node)
                angles.append(angle)

        
        for i in range(len(angList)):
            
            self.rotateTo(angList[i], velocity)
            
            #bruteLidar = self.lidar.getBruteDate()
            distance = self.lidar.getPointRead()[2]
            threshold = 2*self.radius + 1.4*self.ownRadius
            if self.checkDistance(threshold) ==  True:
                
                orientation = self.getAbsoluteOrientation(False)
                   
                coord = AuxiliarFunctions.projectCoord(orientation[2], initCoord, 2*self.radius)
                
                nodes.append(coord)
                angles.append(orientation[2])
                #distances.append(distance)

            time.sleep(0.15)
            
        logNeighbor(nodes,angles,initCoord)

        return nodes,angles

        
    def turnOnRobot(self,velocity,rotationSense):
        if rotationSense == "hor":
            sim.simxSetJointTargetVelocity(self.idClient,self.motorsId[0],velocity,sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(self.idClient,self.motorsId[1],-velocity,sim.simx_opmode_oneshot)
        elif rotationSense == "antihor":
            sim.simxSetJointTargetVelocity(self.idClient,self.motorsId[0],-velocity,sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(self.idClient,self.motorsId[1],velocity,sim.simx_opmode_oneshot)
        elif rotationSense == "front":
            sim.simxSetJointTargetVelocity(self.idClient,self.motorsId[0],velocity,sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(self.idClient,self.motorsId[1],velocity,sim.simx_opmode_oneshot)
        else:
            sim.simxSetJointTargetVelocity(self.idClient,self.motorsId[0],-velocity,sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(self.idClient,self.motorsId[1],-velocity,sim.simx_opmode_oneshot)
        
        
    def rotateTo(self,angle,velocity):
        #Verificar pra qual lado é mais curto girar
        currentOrientation = self.getAbsoluteOrientation(False)
        index =2
        
        diffHor = AuxiliarFunctions.diffAngles(angle,currentOrientation[index] , -1)
        diffAntiHor = AuxiliarFunctions.diffAngles(angle,currentOrientation[index] , 1)

        
        if diffHor < diffAntiHor:
            self.turnOnRobot(velocity,'hor')
            sign = 1
        else:
            self.turnOnRobot(velocity,'antihor')
            sign = -1

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

            
            
    def checkDistance(self,thresholdMax,thresholdMin = 0.1):
        cont = 0
        '''
        lidarCenter = []
        lidarRight = []
        lidarLeft = []
        '''

        #threshold = 2*self.radius + 1.4*self.ownRadius
        #angC = 
        for i in range(6):
            bruteLidar = self.lidar.getBruteDate()
            #distance = self.lidar.getPointRead()[2]

            #Projetar quando da erro
            #Percorrer metades de todos os pontos do lidar
            flagProximity = False
            #for index in range(len(bruteLidar)):
                #angle = 
            ang = 0
            interator = 1
            for index in range(0,len(bruteLidar),2):
                dist = math.sqrt(bruteLidar[index][1]**2 + bruteLidar[index][2]**2)
                if ang < 76 and abs(bruteLidar[index][2])<self.ownRadius + 0.02 and dist>thresholdMin:
                    flagProximity = True
                    #print("Angulo:" + str(ang))
                    #print("distância:" + str(dist))
                    #print("Limiar:" + str(self.ownRadius*math.cos(ang*math.pi/180)+0.05))
                    break
                elif ang>=76 and bruteLidar[index][1] <= thresholdMax and dist > thresholdMin:
                    flagProximity = True
                    #print("Angulo:" + str(ang))
                    #print("distância:" + str(dist))
                    #print("Limiar:" + str(thresholdMax))
                    break
                if ang == 90:
                    interator*=-1
                ang += interator
                
            if flagProximity == True:
                cont+=1
            '''  
            for data in bruteLidar:

                if abs(data[2])<0.01:
                    ang = 1.5708
                else:
                    ang = math.atan(abs(data[1]/data[2]))
                dist = math.sqrt(data[1]**2 + data[2]**2)
                #deg = (ang*180/math.pi)
                #print("angulo em rad:" + str(ang))
                #print("angulo graus:" + str(deg) + " dist:" + str(dist))
                #print("dist plan:" + str(self.ownRadius*math.cos(ang)))
                if ang < 1.3265 and dist < self.ownRadius*math.cos(ang) and dist>thresholdMin:
                    flagProximity = True
                    break
                elif ang>=1.3265 and dist <= thresholdMax and data[1] > thresholdMin:
                    flagProximity = True
                    break
            '''

                    

                #lidarCenter.append(distance)
                #lidarLeft.append(bruteLidar[0][1])
                #lidarRight.append(bruteLidar[-1][1])
                    
            '''
            if (distance <= thresholdMax or bruteLidar[0][1] <= thresholdMax or bruteLidar[-1][1]<= thresholdMax):
                if (distance > thresholdMin or bruteLidar[0][1] > thresholdMin or bruteLidar[-1][1]>thresholdMin):
                    cont+=1
                    lidarCenter.append(distance)
                    lidarLeft.append(bruteLidar[0][1])
                    lidarRight.append(bruteLidar[-1][1])
            '''
            time.sleep(0.1)
            
        if cont<=2:
            return True
        else:
            #logLidar(lidarCenter,lidarLeft,lidarRight)
            return False
            
            
            '''
            if application == 'project'
                if distance >= thresholdMin and bruteLidar[0][1]>= thresholdMin and bruteLidar[-1][1]>= thresholdMin:
                    cont+=1
                else:
                    lidarCenter.append(distance)
                    lidarLeft.append(bruteLidar[0][1])
                    lidarRight.append(bruteLidar[-1][1])
            else:
                if (distance  thresholdMax or bruteLidar[0][1]< thresholdMax or bruteLidar[-1][1]< thresholdMax):
                    
                    cont+=1
                else:
                    lidarCenter.append(distance)
                    lidarLeft.append(bruteLidar[0][1])
                    lidarRight.append(bruteLidar[-1][1])
                
        if cont>=3:
            return True
        else:
            logLidar(lidarCenter,lidarLeft,lidarRight)
            return False
        '''
    '''
    def setOrientation(self,angle):
        returnCode = sim.simxSetObjectOrientation(self.idClient,self.robot,-1,[0,0,angle],sim.simx_opmode_oneshot)
        print(returnCode)
        	
    def setPosition(self,position):
        returnCode = sim.simxSetObjectPosition(self.idClient,self.robot,-1,position,sim.simx_opmode_oneshot)
        print(returnCode)	
    '''
    


        
        
        
        
        
        