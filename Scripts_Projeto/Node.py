# -*- coding: utf-8 -*-
"""
Created on Wed Feb 17 18:39:59 2021

@author: raulf
"""
import math

class Node:
    def __init__(self,coord,neighborhoodCoord,neighborhoodAngle,parent = None,parentAngle = None):
        self.coord = coord
        self.neighborhoodCoord = neighborhoodCoord
        self.neighborhoodAngle = neighborhoodAngle
        self.parent = parent
        self.parentAngle = parentAngle
        self.radius = 0.25
        
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
                
