# -*- coding: utf-8 -*-
"""
Created on Wed Feb 17 18:39:59 2021

@author: raulf
"""

class Node:
    def __init__(self,coord,parent = None,parentAngle = None, parentDist = None):
        self.coord = coord
        self.neighborhoodCoord = None
        self.neighborhoodAngle = None
        self.parent = parent
        self.parentAngle = parentAngle
        self.parentDist = parentDist
        self.visitedIndex = 0
        self.resetFlag = False
        #self.radius = 0.3
        
    def addNeighborhood(self,coordList,angleList):
        self.neighborhoodCoord = coordList
        self.neighborhoodAngle = angleList
        
    def checkNeighborhood(self):
        if self.visitedIndex>=len(self.neighborhoodCoord):
            return False
        else:
            return True
        
    def getNeighbor(self):
        coord = self.neighborhoodCoord[self.visitedIndex]
        angle = self.neighborhoodAngle[self.visitedIndex]
        self.confirmVisitedNeighbor()
        return coord,angle
    
    def confirmVisitedNeighbor(self):
        self.visitedIndex += 1
    
    def resetVisitedIndex(self):
        if self.resetFlag == False:
            self.visitedIndex = 0
            self.resetFlag = True
    
    '''
    def deleteNeighborhoodCoord(self,coord):
        for node in self.neighborhoodCoord:
            diff2 = (coord[0] - node[0])**2 + (coord[1] - node[1])**2
            if math.sqrt(diff2) < self.radius:
                self.neighborhoodCoord.remove(node)
                
    def deleteNeighborhoodAngle(self,coord):
        for node in self.neighborhoodAngle:
            diff2 = (coord[0] - node[0])**2 + (coord[1] - node[1])**2
            if math.sqrt(diff2) < self.radius:
                self.neighborhoodCoord.remove(node)
    '''
                
