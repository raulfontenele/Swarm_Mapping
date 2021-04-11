from Node import Node
import math
from Save import saveDebug

class PathPlanning:

    def __init__(self, maze, node_start, node_goal):
        self.map = maze
        self.node_start = node_start
        self.node_goal = node_goal
        self.openList = []
        self.closeList = []

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
    
    def findNeighborhood(self,node_current):
        #neighbor = []
     
        #coord = [node_current.coord[0],node_current.coord[1]]
        neighborhood = []
        for node in self.map.structMap:
            #print(node["nodeCoord"])
            diff2 = (node["nodeCoord"][0] - node_current.coord[0])**2 + (node["nodeCoord"][1] - node_current.coord[1])**2
            if math.sqrt( diff2) < self.map.radius:
                ##Colocar todos os visinhos na variável neighborhood
                for index in range(len(node["neighborhood"])):
                    neighborhood.append(Node(node["neighborhood"][index]))

        '''
        coorX = node_current.coord[0]
        coorY = node_current.coord[1]
        
        if self.map[coorX + 1][coorY] != 1:
            neighbor.append( Node([coorX + 1,coorY]) )
        if self.map[coorX - 1][coorY] != 1 and coorX - 1 > 0:
            neighbor.append( Node([coorX - 1,coorY]) )
        if self.map[coorX ][coorY + 1] != 1:
            neighbor.append( Node([coorX,coorY + 1]) )    
        if self.map[coorX ][coorY - 1] != 1 and coorY - 1 > 0:
            neighbor.append( Node([coorX,coorY - 1]) ) 
        '''
        '''
        if len(neighborhood) == 0:
            neighborhood.append(node_current.parent)
        '''
            
        return neighborhood
    
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

            if node_current.coord == self.node_goal.coord:
                break
        
            neighborhood = self.findNeighborhood(node_current)

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
                    if ( node_current.gcost + 10 < neighbor.gcost ):                      
                        neighbor.parent = node_current
                        neighbor.calculateGCost()
                        neighbor.calculateFCost()

            if len(self.openList) == 0:
                print("deu merda")
                saveDebug("Deu problema no planejamento de caminho")
                break

        route = []

        ## Mostrar a rota
        while node_current != None:
            route.append(node_current.coord)
            node_current = node_current.parent

        return route

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