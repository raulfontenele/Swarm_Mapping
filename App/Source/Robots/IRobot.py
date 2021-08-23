from abc import ABC, abstractmethod

class IRobot(ABC):
    def __init__(self,communicationApi,robotInfos,radiusZone):
        self.comm = communicationApi
        self.robotInfos = robotInfos
    
    @abstractmethod
    def getAbsoluteOrientation(self,flag):
        pass
    
    @abstractmethod
    def getAbsolutePosition(self,flag):
        pass

    @abstractmethod   
    def turnOnRobot(self,velocityR,velocityL,rotationSense):
        pass