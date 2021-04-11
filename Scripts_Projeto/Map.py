# -*- coding: utf-8 -*-
"""
Created on Wed Feb 17 18:37:37 2021

@author: raulf
"""
import math
from Save import saveCoord,saveEdge,saveMap

class Map:
    def __init__(self,radius):
        #self.initNode = initNode
        self.currentNode = None
        self.visitedList = []
        self.noneVisitedList = []
        self.radius = radius
        self.goalsMap = []
        self.structMap = []
        self.statusMap = []
    
    def initGoalsNode(self,robotId,currentCoord):
        self.goalsMap.append(
            {
                "id":robotId,
                "nextGoal":currentCoord,
                "finalGoal":currentCoord
            })
    def initStatusMap(self,robotId,currentStatus):
        self.statusMap.append(
            {
                "id":robotId,
                "status":currentStatus
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
    
    def addMapPoint(self,currentCoord,neighborhood,angles,robotId):
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
            "angles":       angles,
            "robotId":      robotId
        }
        self.structMap.append(struct)
        saveMap(self.structMap, "map")
        #print("Estrutura do mapa que foi adicionada:")
        #print(struct)
    def checkGoalAnother(self,coord,typeDist):
        '''
            A regra de avaliação consiste em um robô definir seu objetivo, onde o mesmo não deverá
            ser o próximo objetivo ou o final de nenhum outro robô, dependendo da escolha
        '''

        for goal in self.goalsMap:
            if math.sqrt((coord[0] - goal["nextGoal"][0])**2 + (coord[1] - goal["nextGoal"][1])**2) < self.radius and (typeDist == "next" or typeDist == "both"):
                return True
            elif math.sqrt((coord[0] - goal["finalGoal"][0])**2 + (coord[1] - goal["finalGoal"][1])**2) < self.radius and (typeDist == "final" or typeDist == "both"):
                return True
        return False
                    
    def getFreeNeighbor(self,currentNode):
        for mapRow in self.structMap:
            diff2 = (currentNode[0] - mapRow["nodeCoord"][0])**2 + (currentNode[1] - mapRow["nodeCoord"][1])**2
            if math.sqrt(diff2) < self.radius:
                neighborhood =  mapRow["neighborhood"]
                break
        for neighbor in neighborhood:
            if self.checkGoalAnother(neighbor,"next") == False:
                return neighbor
                
                
            
            
    def checkAvailability(self,neighborhood,angles):
        '''
        print("tamanho do troço: " + str(len(neighborhood)) )
        print(neighborhood)
        print(angles)
        '''
        for index in range(len(neighborhood)):
            flag,node = self.checkVisited(neighborhood[index])
            if flag == False and self.checkGoalAnother(neighborhood[index],"both") == False:
                return [neighborhood[index],angles[index]]
    
    def updateGoals(self,nextGoal,finalGoal,robotId):
        for index in range(len(self.goalsMap)):
            if self.goalsMap[index]["id"] == robotId:
                self.goalsMap[index]["nextGoal"] = nextGoal
                self.goalsMap[index]["finalGoal"] = finalGoal
                break
                
    def updateStatus(self,robotId,status):
        for index in range(len(self.statusMap)):
            if self.statusMap[index]["id"] == robotId:
                self.statusMap[index]["status"] = status
    
    def checkAdjNumber(self,currentNode):
        for mapRow in self.structMap:
            diff2 = (currentNode[0] - mapRow["nodeCoord"][0])**2 + (currentNode[1] - mapRow["nodeCoord"][1])**2
            if math.sqrt(diff2) < self.radius:
                return len(mapRow["neighborhood"])
    
    def updateVisitedNode(self):
        for noneVisitedNode in self.noneVisitedList:
            for visitedNode in self.visitedList:
                if math.sqrt((noneVisitedNode[0] - visitedNode[0])**2 + (noneVisitedNode[1] - visitedNode[1])**2) < self.radius:
                    self.noneVisitedList.remove(noneVisitedNode)
                    print("Apagou o nó " + str(noneVisitedNode))
                    break
    
    def getCoordNoneVisitedList(self,coord):
        for noneVisitedNode in self.noneVisitedList:
            if math.sqrt((noneVisitedNode[0] - coord[0])**2 + (noneVisitedNode[1] - coord[1])**2) < self.radius:
                return noneVisitedNode
                
                
            
    
            
        
        