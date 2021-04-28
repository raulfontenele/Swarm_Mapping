from AuxiliarFunctions import AuxiliarFunctions


angulos = [0,30,60,90,120,150,180,210,240,270,300,330,360]
threshold = [0,30,60,90,120,150,180,210,240,270,300,330,360]
for ang in angulos:
    for lim in threshold:
        print("-------- Ângulo: " + str(ang) + "  --- Threshold:" + str(lim) + " ---------")
        errorAngle = abs(AuxiliarFunctions.diffAngleThreshold(ang,lim))
        print(errorAngle)