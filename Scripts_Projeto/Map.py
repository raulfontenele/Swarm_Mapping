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
        self.goalsMap = []
        self.structMap = []
    
    def initGoalsNode(self,robotId,currentCoord):
        self.goalsMap.append(
            {
                "id":robotId,
                "nextGoal":currentCoord,
                "finalGoal":currentCoord
            })
        
        
    def addVisitedNode(self,coord):
        self.visitedList.append(coord)
        
        
    def addNoneVisitedNode(self,listCoord):
        for coord in listCoord:
            flag,node = self.checkVisited(coord)
            if self.checkNoneVisitedList(coord) == False and flag == False:
                self.noneVisitedList.append(coord)
        
    def visitedNode(self,coord):
        for node in self.noneVisitedList:
            diff2 = (coord[0] - node[0])**2 + (coord[1] - node[1])**2
            if math.sqrt(diff2) < self.radius:
                self.noneVisitedList.remove(node)
                break
        self.addVisitedNode(coord)
    
    def checkVisited(self,projectedCoord):
        for node in self.visitedList:
            diff2 = (projectedCoord[0] - node[0])**2 + (projectedCoord[1] - node[1])**2
            if math.sqrt(diff2) <= self.radius:
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
            for mapRow in self.structMap:
                diff2 = (neighbor[0] - mapRow["nodeCoord"][0])**2 + (neighbor[1] - mapRow["nodeCoord"][1])**2
                if math.sqrt(diff2) < self.radius:
                    n_neighborhood.append(mapRow["nodeCoord"])
                    flag = True
                    for nwNeighbor in mapRow["neighborhood"]:
                        diff2 = (nwNeighbor[0] - currentCoord[0])**2 + (nwNeighbor[1] - currentCoord[1])**2
                        if math.sqrt(diff2) < self.radius:
                            #nwNeighbor = currentCoord
                            indexNeighbor = mapRow["neighborhood"].index(nwNeighbor)
                            indexElement = self.structMap.index(mapRow)
                            self.structMap[indexElement]["neighborhood"][indexNeighbor] = currentCoord
                            break
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
    def checkGoalAnother(self,coord):
        '''
            A regra de avaliação consiste em um robô definir seu objetivo, onde o mesmo não deverá
            ser o próximo objetivo e nem o final de nenhum outro robô
        '''

        for goal in self.goalsMap:
            if math.sqrt((coord[0] - goal["nextGoal"][0])**2 + (coord[1] - goal["nextGoal"][1])**2) < self.radius:
                return True
            elif math.sqrt((coord[0] - goal["finalGoal"][0])**2 + (coord[1] - goal["finalGoal"][1])**2) < self.radius:
                return True
        return False
                    
    
    def checkAvailability(self,neighborhood,angles):
        '''
        print("tamanho do troço: " + str(len(neighborhood)) )
        print(neighborhood)
        print(angles)
        '''
        for index in range(len(neighborhood)):
            flag,node = self.checkVisited(neighborhood[index])
            if flag == False and self.checkGoalAnother(neighborhood[index]) == False:
                return [neighborhood[index],angles[index]]
    
    def updateGoals(self,nextGoal,finalGoal,robotId):
        for index in range(len(self.goalsMap)):
            if self.goalsMap[index]["id"] == robotId:
                self.goalsMap[index]["nextGoal"] = nextGoal
                self.goalsMap[index]["finalGoal"] = finalGoal
                
            
    
            
        
        