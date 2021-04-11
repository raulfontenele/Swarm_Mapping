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
            
        #print(floatValues)
        #print(len(floatValues))
        #print("reshaped")
        #print(reshapedMatrix)
    def getDetectedState(self,flag):
        name = "LaserScannerLaser_2D"
        obj = self.getObjectHandle(name)
        print(obj)
        print(self.ObjectHandle)
        if flag == True:
            returnCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=sim.simxReadProximitySensor(self.clientID,self.ObjectHandle,sim.simx_opmode_streaming)
            print(detectionState)
            return detectionState
        else:
            returnCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=sim.simxReadProximitySensor(self.clientID,self.ObjectHandle,sim.simx_opmode_buffer)
            print(detectionState)
            return detectionState
        
    def teste(self):
        returnCode,outInts,outFloats,outStrings,outBuffer=sim.simxCallScriptFunction(self.clientID,"LaserScannerLaser_2D",sim.sim_scripttype_childscript,"sysCall_sensing",[],[],[],0,sim.simx_opmode_blocking)
        print(returnCode)
        print(outInts)
        print(outFloats)
        print(outStrings)
        print(outBuffer)
        print("//=====================================//")
    def teste2(self):
        scan = "LaserScannerJoint_2D"
        sensor = "LaserScannerLaser_2D"
        ldObject = self.getObjectHandle(scan)
        objSen = self.getObjectHandle(sensor)
        #print(self.getPointRead())
        for i in np.arange(-math.pi/2,math.pi/2,math.pi/360).tolist():
            print(self.getPointRead())
            #print("valor setado:" + str(i))

            returnCode = sim.simxSetJointPosition(self.clientID,ldObject,i,sim.simx_opmode_streaming)
            returnCode,position=sim.simxGetJointPosition(self.clientID,ldObject,sim.simx_opmode_streaming)

            #print("posição real" + str(position))
            #print(returnCode)
            returnCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=sim.simxReadProximitySensor(self.clientID,objSen,sim.simx_opmode_buffer)
            v1 = np.array(detectedPoint)
            v2 = np.array(detectedSurfaceNormalVector)
            #print(v1)
            #print(v2)
            time.sleep(0.15)
            #print(v1*v2)
    def teste3(self):

        objname = "sensor1"
        objSen = self.getObjectHandle(objname)
        print("numero do troço:" + str(objSen))
        returnCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=sim.simxReadProximitySensor(self.clientID,objSen,sim.simx_opmode_streaming)
        v1 = np.array(detectedPoint)
        v2 = np.array(detectedSurfaceNormalVector)
        print(v1)
        print(v2)
        print(detectionState)
        