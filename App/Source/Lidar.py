# -*- coding: utf-8 -*-
"""
Created on Fri Feb 12 18:39:46 2021

@author: raulf
"""
import numpy as np
import sim
import math
import time

class Lidar:
    def __init__(self,name,clientID,idFunction):
        self.name = name
        self.clientID = clientID
        self.ObjectHandle = self.getObjectHandle(self.name)

        self.flagBruteData = False
        self.idFunction = idFunction
        #Inicialização das funções
        #self.getPointRead()
        #self.getBruteDate()
        self.getDetectedState(True)

        
    def getObjectHandle(self,name):
        returnCode,handle=sim.simxGetObjectHandle(self.clientID,name,sim.simx_opmode_blocking)
        return handle
    
    
    def getPointRead(self):
        returnCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=sim.simxReadProximitySensor(self.clientID,self.ObjectHandle,sim.simx_opmode_streaming)
        return detectedPoint
        
    def getDetectedState(self,flag):
        if flag == True:
            returnCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=sim.simxReadProximitySensor(self.clientID,self.ObjectHandle,sim.simx_opmode_streaming)
            return detectionState
        else:
            returnCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=sim.simxReadProximitySensor(self.clientID,self.ObjectHandle,sim.simx_opmode_buffer)
            print(detectionState)
            return detectionState

    def getObjectChildHandle(self,parantHandle,childIndex):
        returnCode,childtHandle=sim.simxGetObjectChild(self.clientID,parantHandle,childIndex,sim.simx_opmode_blocking)
        #returnCode,child=sim.simxGetObjectHandle(self.clientID,name,sim.simx_opmode_blocking)
        print(childtHandle)
        return childtHandle

    def getBruteDate(self):
        if self.flagBruteData == False:
            returnCode,signalValue = sim.simxGetStringSignal(self.clientID,'measuredDataAtThisTime' + str(self.idFunction),sim.simx_opmode_streaming) 
            self.flagBruteData = True
        else:
            returnCode,signalValue = sim.simxGetStringSignal(self.clientID,'measuredDataAtThisTime' + str(self.idFunction),sim.simx_opmode_buffer)
            
        floatValues = sim.simxUnpackFloats(signalValue)
        try:
            matrix = np.array(floatValues)
            lines = int(len(floatValues)/3)
            #lin = (self.angle + 1)*self.density
            reshapedMatrix = np.reshape(matrix,[lines,3])
        except:
            reshapedMatrix = 0
            print("Deu merda nos dados brutos")
            print(len(floatValues))
            print(floatValues)
        
        return reshapedMatrix