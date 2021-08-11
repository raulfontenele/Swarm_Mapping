# -*- coding: utf-8 -*-
"""
Created on Tue Aug 10 17:38:25 2021

@author: raulf
"""

import sim

class CommunicationAPI:
    def __init__(self, ip, comPort):
        self.ip = ip
        self.comPort = comPort
        #self.Clear()
        self.clientId = sim.simxStart(self.ip, self.comPort, True, True, 5000, 5)
        
    def Initialize(self):
        sim.simxStartSimulation(self.clientId, sim.simx_opmode_oneshot_wait)

    def Stop(self):
        sim.simxStopSimulation(self.clientId, sim.simx_opmode_oneshot_wait)

    def Dispose(self):
        sim.simxFinish(self.clientId)

    def SucessConnect(self):
        return True if self.clientId != -1 else False
    
    def Clear(self):
        sim.simxFinish(-1)

    def Pause(self, condition):
        sim.simxPauseCommunication(self.clientId, condition)