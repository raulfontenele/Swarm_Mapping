# -*- coding: utf-8 -*-
"""
Created on Wed Feb 17 18:37:37 2021

@author: raulf
"""
import math

class Map:
    def __init__(self,initNode,radius):
        self.initNode = initNode
        self.currentNode = None
        self.visitedList = []
        self.noneVisitedList = []
        self.radius = radius
        self.edgeMap = []
        self.matrixMap = []
    
    #def addNode(self,parent):
        #parent.
        
    def addVisitedNode(self,coord):
        self.visitedList.append(coord)
        self.matrixMap.append([self.currentNode.coord,coord])
        
    def addNoneVisitedNode(self,listCoord):
        for coord in listCoord:
            if self.checkNoneVisitedList(coord) == False and self.checkVisited(coord,False) == False:
                self.noneVisitedList.append(coord)
        
    def visitedNode(self,coord):
        for node in self.noneVisitedList:
            diff2 = (coord[0] - node[0])**2 + (coord[1] - node[1])**2
            if math.sqrt(diff2) < self.radius:
                self.noneVisitedList.remove(node)
                break
        self.addVisitedNode(coord)
    
    def checkVisited(self,projectedCoord,flagControl):
        for node in self.visitedList:
            diff2 = (projectedCoord[0] - node[0])**2 + (projectedCoord[1] - node[1])**2
            if math.sqrt(diff2) <= self.radius:
                if flagControl == True:
                    print("Aresta adicionada")
                    self.edgeMap.append([self.currentNode.coord,node])
                return True
        return False
    
    def checkNoneVisitedList(self,coord):
        for node in self.noneVisitedList:
            diff2 = (coord[0] - node[0])**2 + (coord[1] - node[1])**2
            if math.sqrt(diff2) <= self.radius:
                return True
        return False
    
            
        
        