# -*- coding: utf-8 -*-
"""
Created on Wed Feb 17 18:37:37 2021

@author: raulf
"""
import math
from Save import saveCoord,saveEdge,saveMap,saveDebug,saveLists
import json
import csv

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
        self.stepMap = []
    
    
    def initGoalsNode(self,robotId,currentCoord):
        self.goalsMap.append(
            {
                "id":robotId,
                "currentPosition":currentCoord,
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
            if self.checkNoneVisitedList(coord) == False and self.checkVisited(coord) == False:
                self.noneVisitedList.append(coord)
        
    def visitedNode(self,coord):
        try:
            for node in self.noneVisitedList:
                diff2 = (coord[0] - node[0])**2 + (coord[1] - node[1])**2
                if math.sqrt(diff2) < self.radius:
                    self.noneVisitedList.remove(node)
                    break
            self.addVisitedNode(coord)

            if self.checkNoneVisitedList(coord) == True:
                raise Exception("Continua na lista de nós não visitados")
            if self.checkVisited(coord) == False:
                raise Exception("Ainda não está na lista de nós visitados")
                
        except Exception as inst:
            string = inst.args[0]
            saveDebug(string)
    
    def checkVisited(self,projectedCoord):
        for node in self.visitedList:
            diff2 = (projectedCoord[0] - node[0])**2 + (projectedCoord[1] - node[1])**2
            if math.sqrt(diff2) <= self.radius:
                return True
        return False
    
    def checkNoneVisitedList(self,coord):
        for node in self.noneVisitedList:
            diff2 = (coord[0] - node[0])**2 + (coord[1] - node[1])**2
            if math.sqrt(diff2) <= self.radius:
                return True
        return False
    
    def addMapPoint(self,currentCoord,neighborhood,angles,robotId):
        #Verificar se o ponto já existe e, em caso positivo, colocar o ponto que já existe no lugar do projetado

        #print("A quantidade de visinhos a entrar no  addPoint é " + str(len(neighborhood)))
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
        
        #print("A quantidade real de visinhos a entrar no  addPoint é " + str(len(n_neighborhood)))   
        struct = {
            "nodeCoord":    currentCoord,
            "neighborhood": n_neighborhood,
            "angles":       angles,
            "robotId":      robotId
        }
        self.structMap.append(struct)
        saveMap(self.structMap, "map")

    def checkGoalAnother(self,coord,typeDist,robotId):
        '''
            A regra de avaliação consiste em um robô definir seu objetivo, onde o mesmo não deverá
            ser o próximo objetivo ou o final de nenhum outro robô, dependendo da escolha
            Não poderá ser ele mesmo
        '''

        for goal in self.goalsMap:
            if math.sqrt((coord[0] - goal["nextGoal"][0])**2 + (coord[1] - goal["nextGoal"][1])**2) < self.radius and (typeDist == "next" or typeDist == "both"):
                if goal["id"] != robotId:
                    return True
            elif math.sqrt((coord[0] - goal["finalGoal"][0])**2 + (coord[1] - goal["finalGoal"][1])**2) < self.radius and (typeDist == "final" or typeDist == "both"):
                if goal["id"] != robotId:
                    return True
        return False
                    
    def getFreeNeighbor(self,currentNode,robotId):
        for mapRow in self.structMap:
            diff2 = (currentNode[0] - mapRow["nodeCoord"][0])**2 + (currentNode[1] - mapRow["nodeCoord"][1])**2
            if math.sqrt(diff2) < self.radius:
                neighborhood =  mapRow["neighborhood"]
                break
        for neighbor in neighborhood:
            if self.checkGoalAnother(neighbor,"next",robotId) == False and self.checkOccupiedNode(neighbor,robotId) == False:
                return neighbor
                
    def checkAvailability(self,neighborhood,angles,robotId):
        freeNeighborhood = []
        #Checar se existem visinhos disponíveis para serem visitados
        for index in range(len(neighborhood)):
            #flag,node = self.checkVisited(neighborhood[index])
            if self.checkVisited(neighborhood[index]) == False and self.checkGoalAnother(neighborhood[index],"both",robotId) == False:
                freeNeighborhood.append(neighborhood[index])
                #return [neighborhood[index],angles[index]]
        return freeNeighborhood
    
    def updateGoals(self,currentNode,nextGoal,finalGoal,robotId):
        for index in range(len(self.goalsMap)):
            if self.goalsMap[index]["id"] == robotId:
                self.goalsMap[index]["currentPosition"] = currentNode
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
        return 0
    
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
    
    def checkOccupiedNode(self,coord,robotId):
        for robot in self.goalsMap:
            if robot["id"] != robotId:
                if math.sqrt( (robot["currentPosition"][0]-coord[0])**2 + (robot["currentPosition"][1]-coord[1])**2 ) < self.radius:
                    return True
        return False
    
    def addStepMap(self,robotId,coord):
        step = {
            "id":robotId,
            "coord":coord,
            }
        self.stepMap.append(step)
        saveMap(self.stepMap, "stepMap")
    
    
    def getStatusRobot(self,coord):
        
        #Procurar qual o robô está ocupando aquele espaço e depois procurar qual o status do robô
        
        robotId = ''
        for robot in self.goalsMap:
            if math.sqrt( (robot["currentPosition"][0]-coord[0])**2 + (robot["currentPosition"][1]-coord[1])**2 ) < self.radius:
                robotId = robot["id"]
        
        if robotId != '':
            for status in self.statusMap:
                if status["id"] == robotId:
                    return status["status"]
        else:
            return "moving"

    def getRobotsPosition(self):
        robotsPosition = []
        for nodes in self.goalsMap:
            robotsPosition.append(nodes["currentPosition"])
        return robotsPosition
    
    '''
    #---------------------------------------------------------------------#    
    def checkVisitedStruct(self,projectedCoord):
        for struct in self.structMap:
            diff2 = (projectedCoord[0] - struct["nodeCoord"][0])**2 + (projectedCoord[1] - struct["nodeCoord"][1])**2
            if math.sqrt(diff2) <= self.radius:
                return True
        return False
    #---------------------------------------------------------------------#
    '''
    '''
    def checkVisitedCoord(self,projectedCoord):
        file = open('map.txt','r')
        lines = file.readlines()
    
        for line in lines:
            line = line.replace("'",'"')
            struct =json.loads(line)
            diff2 = (projectedCoord[0] - struct["nodeCoord"][0])**2 + (projectedCoord[1] - struct["nodeCoord"][0])**2
            if math.sqrt(diff2) <= self.radius:
                file.close()
                return True
        file.close()
        return False
    '''


            

    
                
                
                
            
    
            
        
        