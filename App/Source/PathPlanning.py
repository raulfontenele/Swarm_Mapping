from Node import Node
import math
from Save import saveDebug,saveMap

class PathPlanning:

    def __init__(self, maze, node_start, node_goal):
        self.map = maze
        self.node_start = node_start
        self.node_goal = node_goal
        self.openList = []
        self.closeList = []
        #saveMap(self.map.structMap, "mapPath")

    def calculateHCost(self,node_current):
        h = abs(node_current.coord[0] - self.node_goal.coord[0]) + abs(node_current.coord[1] - self.node_goal.coord[1])
        return h

    def checkOpenList(self,node_current):
        for node in self.openList:
            if node.coord == node_current.coord:
                return True

        return False

    def checkCloseList(self,node_current):
        for node in self.closeList:
            if node.coord == node_current.coord:
                return True

        return False
    
    def findNeighborhood(self,node_current, goal_node_coord):
        neighborhood = []
        for node in self.map.structMap:
            diff2 = (node["nodeCoord"][0] - node_current.coord[0])**2 + (node["nodeCoord"][1] - node_current.coord[1])**2
            if math.sqrt( diff2) < self.map.radius:
                ##Colocar todos os visinhos na variável neighborhood
                for index in range(len(node["neighborhood"])):
                    if self.checkValidNeighbor(node["neighborhood"][index]) == True or node["neighborhood"][index] == goal_node_coord:
                        neighborhood.append(Node(node["neighborhood"][index]))
                    else:
                        saveDebug("Coordenada excluida:: " + str(node["neighborhood"][index]))
                        
            
        return neighborhood

    def checkValidNeighbor(self,node_current):
        #print(node_current)
        for node in self.map.structMap:
            diff2 = (node["nodeCoord"][0] - node_current[0])**2 + (node["nodeCoord"][1] - node_current[1])**2
            if math.sqrt(diff2) < self.map.radius:
                return True
        return False

    
    def validateNeighborhood(self,neighborhood):
        validList = []
        for neighbor in neighborhood:
            if self.checkCloseList(neighbor) == False:
                validList.append(neighbor)
        return validList


    def AStarAlgorithm(self):

        self.openList.append(self.node_start)

        while True:
            self.openList = sorted(self.openList,key=Node.getFcode)
            node_current = self.openList[0]
            self.closeList.append(self.openList[0])
            self.openList.pop(0)

            if [round(i,2) for i in node_current.coord] == [round(i,2) for i in self.node_goal.coord]:
                break
        
            neighborhood = self.findNeighborhood(node_current,self.node_goal.coord)

            for neighbor in neighborhood:
                if self.checkCloseList(neighbor):
                    continue
                elif self.checkOpenList(neighbor) == False:
                    neighbor.parent = node_current
                    neighbor.hcost = self.calculateHCost(neighbor)
                    neighbor.calculateGCost()
                    neighbor.calculateFCost()
                    self.openList.append(neighbor)
                elif self.checkOpenList(neighbor) :
                    neighbor = [node for node in self.openList if node.coord == neighbor.coord][0]
                    if ( node_current.gcost + 0.5 < neighbor.gcost ):                      
                        neighbor.parent = node_current
                        neighbor.calculateGCost()
                        neighbor.calculateFCost()

            if len(self.openList) == 0:
                print("deu merda")
                saveDebug("Deu problema no planejamento de caminho com origem em: " + str(self.node_start.coord) + " e objetivo: " + str(self.node_goal.coord))
                return [],False
                break

        route = []

        ## Mostrar a rota
        while node_current != None:
            route.append(node_current.coord)
            node_current = node_current.parent

        return route,True

    def DepthFirstSearch(self):

        node_current = self.node_start

        while True:
                
            if node_current.coord == self.node_goal.coord:
                break
            
            if self.checkCloseList(node_current) == False:
                self.closeList.append(node_current)

            neighborPossible = self.findNeighborhood(node_current)
            neighborhood = self.validateNeighborhood(neighborPossible)
            
            if len(neighborhood) > 0:
                next_node = neighborhood[0]
            else :
                next_node = None

            if next_node == None and (node_current.coord != self.closeList[0].coord):
                node_current = node_current.parent
            
            elif next_node == None and (node_current.coord == self.closeList[0].coord):
                print("Não foi possível determinar uma rota válida!")
                return 0

            else:
                next_node.parent = node_current
                node_current = next_node

        route = []

        ## Mostrar a rota
        while node_current != None:
            route.append(node_current.coord)
            print(node_current.coord)
            node_current = node_current.parent

        return route