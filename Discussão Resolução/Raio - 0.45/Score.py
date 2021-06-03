import json
from AuxiliarFunctions import AuxiliarFunctions

def getScore(thresholdDistance):
    #Get all lines and the length of node map
    file = open('map.txt','r')
    lines = file.readlines()
    lenNodeMap = len(lines)


    for i in [0,0.05,0.1,0.15,0.2,0.25,0.3,0.35,0.4,0.45,0.5,0.55,0.6,0.65,0.7,0.75,0.8,0.85,0.9,0.95,1]:
        error = 0
        errorAngleList = []
        
        for line in lines:
            line = line.replace("'",'"')
            struct = json.loads(line)
            for index in range(len(struct["neighborhood"])):
                #Calcule real angle and distance between two points
                distance,angle = AuxiliarFunctions.CalcAngleDistance(struct["nodeCoord"],struct["neighborhood"][index])
                errorAngle = abs(AuxiliarFunctions.diffAngleThreshold(struct["angles"][index],angle))
                errorAngleList.append(errorAngle)
                #print(errorAngle)
                errorAngleNormalize = errorAngle/180
                errorDistanceNormalize = abs(distance-thresholdDistance)/thresholdDistance
                error += (errorAngleNormalize*i + errorDistanceNormalize*(1-i))
        
        #Mean error
        meanError = error/lenNodeMap
        #print("Configuração-> Angulo: " + str(i) + " Distância:" + str(1-i))
        #print("Mean error: " + str(meanError))
        print(meanError)

    print("Max error angle = " + str(max(errorAngleList)))

def main():
    thresholdDistance = 0.9
    getScore(thresholdDistance)

main()