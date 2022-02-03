import json
import matplotlib.pyplot as plt
import time
import numpy as np

#from analysis import analysis

def graph():
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
    
    nw_coord1 = np.array(coord1)
    #plt.subplot(3,1,1)
    plt.figure()
    plt.scatter(nw_coord1[:,0],nw_coord1[:,1], s = 1000, marker='h', c= "#ff0000")

    nw_coord2 = np.array(coord2)
    #plt.subplot(3,1,2)
    plt.figure()
    plt.scatter(nw_coord2[:,0],nw_coord2[:,1], s = 1000, marker='h', c= "#00ff00")

    nw_coord3 = np.array(coord3)
    #plt.subplot(3,1,3)
    plt.figure()
    plt.scatter(nw_coord3[:,0],nw_coord3[:,1], s = 1000, marker='h', c= "#0000ff")

    
    struct = analysis()
    print(struct)
    color = ['#ccccff','#4d4dff','#0000cc','#c61aff','#8600b3','#ff3333','#e60000','#990000']

    plt.figure()
    for node in struct:
        plt.scatter(node['coord'][0],node['coord'][1], s = 1000, marker='h', c = color[node['number']-1])

    '''
    plt.scatter(nw_coord1[:,0],nw_coord1[:,1], s = 1000, marker='h', c= "#ff0000")
    plt.scatter(nw_coord2[:,0],nw_coord2[:,1], s = 1000, marker='h', c= "#00ff00")
    plt.scatter(nw_coord3[:,0],nw_coord3[:,1], s = 1000, marker='h', c= "#0000ff")
    '''
    #print(analysis())
    



    plt.grid()
    plt.show()

def analysis():
    file = open('stepMap.txt','r')
    lines = file.readlines()

    radius = 0.25

    #ids  = [1,2,3]
    coordinates = []
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
                break
        if flag == False:
            coordUnique.append({
                "coord" : struct["coord"],
                "number" : 1
            })
        coordinates.append(struct["coord"])
    listSorted = sorted(coordUnique, key=lambda x: x["number"])
    #print(listSorted[-1])
        

    with open('nodes.txt', 'w', encoding='utf-8') as file:
        for node in listSorted:
            file.write(str(node) + '\n')
    #print(coordUnique)
    return listSorted 



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
    '''
    file = open('coord.csv','r')
    lines = file.readlines()
    plt.figure(1)
    for line in lines:
        values = line.split(',')
        plt.scatter(float(values[0]),float(values[1]), s = 1000, marker='h', c= "#0000cc")
    file.close()
    '''
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

fun2()
#graph()
#analysis()
#fun3()