import math

class Ordering:
    def __init__(self, map, robot):
        self.map = map
        self.robot = robot

    def Fifo(self, unorderedList):
        return unorderedList
    
    def Minimum(self, unorderedList):
        point = self.robot.getAbsolutePosition(False)
        distanceList = []
        for node in unorderedList:
            distance = math.sqrt( (node[0] - point[0])**2 +  (node[1] - point[1])**2)
            distanceList.append(distance)
        
        zipped_lists = zip(distanceList, unorderedList)
        sorted_zipped_lists = sorted(zipped_lists)

        sorted_list = [element for _, element in sorted_zipped_lists]
        
        return sorted_list
    
    def Maximum(self, unorderedList):
        robotsPosition = self.map.getRobotsPosition()
        distanceList = []
        for node in unorderedList:
            distance = 0
            for robotCoord in robotsPosition:
                distance += math.sqrt( (node[0] - robotCoord[0])**2 +  (node[1] - robotCoord[1])**2)
            distanceList.append(distance)

        zipped_lists = zip(distanceList, unorderedList)

        sorted_zipped_lists = sorted(zipped_lists, reverse=True)

        sorted_list = [element for _, element in sorted_zipped_lists]

        return sorted_list
