import json
import matplotlib.pyplot as plt
import time
import numpy as np

#from analysis import analysis

def graph(numRobots):
    file = open('stepMap.txt','r')
    lines = file.readlines()
    ids  = [1,2,3]
    coord1 = []
    coord2 = []
    coord3 = []

    for line in lines:
        line = line.replace("'",'"')
        struct =json.loads(line)
        if struct["id"] == 1:
            coord1.append(struct["coord"])
        elif struct["id"] == 2:
            coord2.append(struct["coord"])
        elif struct["id"] == 3:
            coord3.append(struct["coord"])
    color = ["#ff0000","#00ff00","#0000ff"]
    
    coordinates = [np.array(coord1),np.array(coord2),np.array(coord3)]

    nw_coord1 = np.array(coord1)
    #plt.subplot(3,1,1)
    for i in range(numRobots):
        plt.figure()
        plt.scatter(coordinates[i][:,0],coordinates[i][:,1], s = 1000, marker='h', c= color[i])
        plt.grid()


    
    struct,coordinate,qtd = analysis()
    color = ['#ccccff','#4d4dff','#0000cc','#c61aff','#8600b3','#ff3333','#e60000','#990000']

    plt.figure()
    coord = np.array(coordinate)
    qtdArray = np.array(qtd)
    
    plt.scatter(-coord[:,1],coord[:,0], s = 1000, c = qtdArray ,marker='h', cmap="plasma")
    plt.colorbar()

    file = open('map.txt','r')
    lines = file.readlines()

    for line in lines:
        line = line.replace("'",'"')
        struct =json.loads(line)
        for neighbor in struct["neighborhood"]:
            plt.plot( [-struct["nodeCoord"][1],-neighbor[1]],[struct["nodeCoord"][0],neighbor[0]],c="#1e962c")
            
    plt.title("Cenário 2 - Três robôs - Minimum")
    plt.grid()
    plt.show()

def analysis():
    file = open('stepMap.txt','r')
    lines = file.readlines()

    radius = 0.5

    #ids  = [1,2,3]
    coordinates = []
    qtd = []
    coordUnique = []
    for line in lines:
        line = line.replace("'",'"')
        struct =json.loads(line)
        
        #coordinates.append(struct["coord"])
        flag = False
        for node in coordUnique:
            if (  (struct["coord"][0] - node["coord"][0])**2 + (struct["coord"][1] - node["coord"][1])**2   )**(1/2) < radius:
                flag =  True
                index = coordUnique.index(node)
                coordUnique[index]["number"]+=1
                qtd[index]+=1
                break
        if flag == False:
            coordUnique.append({
                "coord" : struct["coord"],
                "number" : 1
            })
            coordinates.append(struct["coord"])
            qtd.append(1)
    listSorted = sorted(coordUnique, key=lambda x: x["number"])
    #print(listSorted[-1])
        

    with open('nodes.txt', 'w', encoding='utf-8') as file:
        for node in listSorted:
            file.write(str(node) + '\n')
    #print(coordUnique)
    return listSorted,coordinates,qtd 



def fun2():
    file = open('map.txt','r')
    lines = file.readlines()

    plt.figure(0)
    for line in lines:
        line = line.replace("'",'"')
        struct =json.loads(line)
        for neighbor in struct["neighborhood"]:
            plt.plot( [struct["nodeCoord"][0],neighbor[0]],[struct["nodeCoord"][1],neighbor[1]])
            plt.scatter(struct["nodeCoord"][0],struct["nodeCoord"][1], s = 1000, marker='h', c= "#0000cc")
    plt.grid()
    file.close()
    file = open('coord.csv','r')
    lines = file.readlines()
    plt.figure(1)
    for line in lines:
        values = line.split(',')
        plt.scatter(float(values[0]),float(values[1]), s = 1000, marker='h', c= "#0000cc")
    file.close()
    plt.grid()
    plt.show()

def fun3():
    file = open('coord.csv','r')
    lines = file.readlines()

    for line in lines:
        values = line.split(',')
        #print(values)
        #print(values[0])
        plt.scatter(float(values[0]),float(values[1]), s = 1000, marker='h', c= "#0000cc")
    
    plt.grid()
    plt.show()

#fun2()
graph(3)
#analysis()
#fun3()