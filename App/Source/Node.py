
class Node:
    
    def __init__(self,coordenates,angle = None):
        self.coord = coordenates
        self.angle = angle
        self.parent = None
        self.fcost = 0
        self.gcost = 0
        self.hcost = 0

    def calculateFCost(self):
        self.fcost = self.gcost + self.hcost

    def calculateGCost(self):
        self.gcost = self.parent.gcost + ((self.parent.coord[0] - self.coord[0])**2 + (self.parent.coord[1] - self.coord[1])**2)**(1/2)
    
    def getFcode(self):
        return self.fcost
