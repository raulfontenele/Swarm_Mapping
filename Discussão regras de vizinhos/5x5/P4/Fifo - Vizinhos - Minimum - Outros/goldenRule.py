from AuxiliarFunctions import AuxiliarFunctions
import matplotlib.pyplot as plt

def calcNeighbor(nodeCoord):
    angles = [0,60,120,180,240,300]
    for ang in angles:
        neighborCoord = AuxiliarFunctions.projectCoord(ang,nodeCoord,0.5)
        neighborCoord = roundCoord(neighborCoord)

        if checkParameters(neighborCoord) == True and checkInsideList(neighborCoord) == False:
            nodes.append(neighborCoord)
        

def checkParameters(coord):
    if (coord[0]> limits['x_min'] and coord[0] < limits['x_max']) and (coord[1]> limits['y_min'] and coord[1] < limits['y_max']):
        return True
    else:
        return False

def checkInsideList(coord):
    num = nodes.count(coord)
    if num > 0:
        return True
    else:
        return False

def roundCoord(coord):
    for i in range(2):
        coord[i] = round(coord[i],3)
    return coord


initCoord = [-2.0000603199005127, 2.024958610534668,0]
angles = [0,60,120,180,240,300]
limits = {
    "x_max": 2.5,
    "x_min": -2.5,
    "y_max": 2.5,
    "y_min": -2.5
}
nodes = []

initCoord = roundCoord(initCoord)
nodes.append(initCoord)

for node in nodes:
    calcNeighbor(node)
    plt.scatter( node[0],node[1], s = 1000, marker='H', c="#1e962c")

print("Number of nodes: {}",len(nodes))
plt.show()





