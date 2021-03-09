# -*- coding: utf-8 -*-
"""
Created on Fri Feb 12 18:39:46 2021

@author: raulf
"""
import numpy as np
import sim

class Lidar:
    def __init__(self,name,clientID,angle,density):
        self.name = name
        self.clientID = clientID
        self.ObjectHandle = self.getObjectHandle(self.name)
        self.angle = angle
        self.density = density
        
        #Inicialização das funções
        self.getPointRead()
        self.getBruteDate()

        
    def getObjectChildHandle(self,parantHandle,childIndex):
        returnCode,childtHandle=sim.simxGetObjectChild(self.clientID,parantHandle,childIndex,sim.simx_opmode_blocking)
        #returnCode,child=sim.simxGetObjectHandle(self.clientID,name,sim.simx_opmode_blocking)
        print(childtHandle)
        return childtHandle
    
    def getObjectHandle(self,name):
        returnCode,handle=sim.simxGetObjectHandle(self.clientID,name,sim.simx_opmode_blocking)
        return handle
    
    
    def getPointRead(self):
        returnCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=sim.simxReadProximitySensor(self.clientID,self.ObjectHandle,sim.simx_opmode_streaming)
        #print("coordenadas dos pontos")
        #print(detectedPoint)

        return detectedPoint
        
    def getBruteDate(self):
        returnCode,signalValue = sim.simxGetStringSignal(self.clientID,'measuredDataAtThisTime',sim.simx_opmode_streaming) 
        floatValues = sim.simxUnpackFloats(signalValue)
        try:
            matrix = np.array(floatValues)
            lin = (self.angle + 1)*self.density
            reshapedMatrix = np.reshape(matrix,[lin,3])
        except:
            reshapedMatrix = 0
        
        return reshapedMatrix
            
        #print(floatValues)
        #print(len(floatValues))
        #print("reshaped")
        #print(reshapedMatrix)

        