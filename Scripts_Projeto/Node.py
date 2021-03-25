
class Node:
    
    def __init__(self,coordenates,angle = None):
        self.coord = coordenates
        self.angle = angle
        self.parent = None
        self.fcost = None
        self.gcost = None
        self.hcost = None

    def calculateFCost(self):
        self.fcost = self.gcost + self.hcost

    def calculateGCost(self):
        self.gcost = self.parent.gcost + 10
    
    def getFcode(self):
        return self.fcost
