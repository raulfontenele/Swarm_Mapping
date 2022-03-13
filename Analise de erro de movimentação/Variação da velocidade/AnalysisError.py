import matplotlib.pyplot as plt

combinations = [
    [0,0],[0,1],[1,0],[1,1],[2,0]
]

initCoord = [-4.525, 0]

fig, axs = plt.subplots(3, 2)

for j in range(5):

    print("Configuração " + str(j+5))

    for i in range(5):
        with open("Config " + str(j+5)+ "/Execução " + str(i+1) +".txt","r") as file:
            lines = file.readlines()

        distance = []
        error = []

        
        
        coord_x = float(lines[-1].split("::")[2].split(",")[0].replace("[",""))
        coord_y = float(lines[-1].split("::")[2].split(",")[1])

        print(initCoord[1] - coord_y)
        '''
        for line in lines:
            coord_x = float(line.split("::")[2].split(",")[0].replace("[",""))
            coord_y = float(line.split("::")[2].split(",")[1])

            error.append(initCoord[1] - coord_y)
            distance.append(coord_x  - initCoord[0])

        x = combinations[j][0]
        y = combinations[j][1]
        axs[x, y].plot(distance,error)
        '''
'''  
plt.grid()
plt.show()
'''
    