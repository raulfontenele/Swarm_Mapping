# -*- coding: utf-8 -*-
"""
Created on Wed Feb 17 18:37:37 2021

@author: raulf
"""
import math

class Map:
    def __init__(self,radius):
        #self.initNode = initNode
        self.currentNode = None
        self.visitedList = []
        self.noneVisitedList = []
        self.radius = radius
        self.edgeMap = []
        self.matrixMap = []
        self.structMap = []
    
    #def addNode(self,parent):
        #parent.
        
    def addVisitedNode(self,coord):
        self.visitedList.append(coord)
        #self.matrixMap.append([self.currentNode.coord,coord])
        
    def addNoneVisitedNode(self,listCoord):
        for coord in listCoord:
            flag,node = self.checkVisited(coord,False)
            if self.checkNoneVisitedList(coord) == False and flag == False:
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
                return True,node
        return False,None
    
    def checkNoneVisitedList(self,coord):
        for node in self.noneVisitedList:
            diff2 = (coord[0] - node[0])**2 + (coord[1] - node[1])**2
            if math.sqrt(diff2) <= self.radius:
                return True
        return False
    
    def addMapPoint(self,currentCoord,neighborhood,angles):
        #Verificar se o ponto já existe e, em caso positivo, colocar o ponto que já existe no lugar do projetado
        '''
            Precisa ser modificado
        '''
        n_neighborhood = []
        for neighbor in neighborhood:
            flag = False
            for nodeVisited in self.visitedList:
                diff2 = (neighbor[0] - nodeVisited[0])**2 + (neighbor[1] - nodeVisited[1])**2
                if math.sqrt(diff2) < self.radius:
                    n_neighborhood.append(nodeVisited)
                    flag = True
                    break
            if flag == False:
                n_neighborhood.append(neighbor)
            
            
        struct = {
            "nodeCoord":    currentCoord,
            "neighborhood": n_neighborhood,
            "angles":       angles
        }
        self.structMap.append(struct)
        #print("Estrutura do mapa que foi adicionada:")
        #print(struct)
    
    def checkAvailability(self,neighborhood,angles):
        '''
        print("tamanho do troço: " + str(len(neighborhood)) )
        print(neighborhood)
        print(angles)
        '''
        
        for index in range(len(neighborhood)):
            flag,node = self.checkVisited(neighborhood[index],False)
            if flag == False:
                return [neighborhood[index],angles[index]]
            
    
            
        
        