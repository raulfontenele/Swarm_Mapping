import json
from AuxiliarFunctions import AuxiliarFunctions

def getScore(thresholdDistance):
    #Get all lines and the length of node map
    file = open('map.txt','r')
    lines = file.readlines()
    lenNodeMap = len(lines)

    error = 0

    for line in lines:
        line = line.replace("'",'"')
        struct = json.loads(line)
        for index in range(len(struct["neighborhood"])):
            #Calcule real angle and distance between two points
            distance,angle = AuxiliarFunctions.CalcAngleDistance(struct["nodeCoord"],struct["neighborhood"][index])
            errorAngle = AuxiliarFunctions.diffAngleThreshold(struct["angles"][index])
            print(errorAngle)
            errorAngleNormalize = errorAngle/180
            errorDistanceNormalize = abs(distance-thresholdDistance)/thresholdDistance
            error += (errorAngleNormalize*0.7 + errorDistanceNormalize*0.3)
    
    #Mean error
    meanError = error/lenNodeMap
    print(meanError)


def main():
    thresholdDistance = 0.5
    getScore(thresholdDistance)

main()