# -*- coding: utf-8 -*-
"""
Created on Sun Feb 14 08:07:17 2021

@author: raulf
"""

import sim

class Gyroscope:
    def __init__(self,name,clientID):
        self.name = name
        self.clientID = clientID
        self.ObjectHandle = self.getObjectHandle(self.name)
        self.forceSensorObject = self.getObjectChildHandle(self.ObjectHandle,1)
        self.massObject = self.getObjectChildHandle(self.forceSensorObject,0)
        self.mass = self.getObjectMass(self.massObject)
        #self.getGyroscopeRead()
        
    
    def getObjectChildHandle(self,parantHandle,childIndex):
        returnCode,childtHandle=sim.simxGetObjectChild(self.clientID,parantHandle,childIndex,sim.simx_opmode_blocking)
        #returnCode,child=sim.simxGetObjectHandle(self.clientID,name,sim.simx_opmode_blocking)
        return childtHandle
    
    def getObjectHandle(self,name):
        returnCode,handle=sim.simxGetObjectHandle(self.clientID,name,sim.simx_opmode_blocking)
        return handle
    
    def getObjectMass(self,objectNumber):
        ##Resolver para usar a constante
        result,mass=sim.simxGetObjectFloatParameter(self.clientID,objectNumber,3005,sim.simx_opmode_blocking)
        return mass
    
    def getGyroscopeRead(self):
        returnCode,state,forceVector,torqueVector=sim.simxReadForceSensor(self.clientID,self.forceSensorObject,sim.simx_opmode_streaming)
        acceleration = []
        
        for i in range(len(forceVector)):
            acceleration.append(forceVector[i]/self.mass)

        return acceleration
    
    def getGyroscoperReadVrep(self):
        returnCode,signalValue = sim.simxGetStringSignal(self.clientID,'GyroscopeVrep',sim.simx_opmode_streaming) 
        floatValues = sim.simxUnpackFloats(signalValue)
        print(floatValues)