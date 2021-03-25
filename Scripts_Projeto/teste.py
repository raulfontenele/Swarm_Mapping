import math
from AuxiliarFunctions import AuxiliarFunctions

rota = [[-1.3788187926871087, 1.564795411704271, 0.03546711429953575], [-1.598353852530709, 1.9963230449265557, 0.03546661138534546], [-1.0928730412028878, 1.9576109316730976, 0.035460442304611206], [-0.8673749707398932, 1.4990560982091348, 0.03546451777219772], [-1.1208583116531372, 1.0680732727050781, 0.03546451777219772]]
'''
for i in range(len(rota)-1):
    deltaX = rota[i][0] - rota[i+1][0]
    deltaY = rota[i][1] - rota[i+1][1]
    
    #dist = math.sqrt(deltaX**2 + deltaY**2)
    dist,angle = AuxiliarFunctions.CalcAngleDistance(rota[i],rota[i+1])
    print("//===========================//")
    print(dist)
    print(angle)
    print("//===========================//")
'''

a = [1,2,3]
c = 4

def funcao():
    return 12


if funcao() == 12:
    print("deu bom")
else:
    print("num deu")