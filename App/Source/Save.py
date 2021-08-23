# -*- coding: utf-8 -*-
"""
Created on Mon Mar  8 22:30:26 2021

@author: raulf
"""
import numpy as np
import json

def saveCoord(coordList,name):
    np.savetxt("./Logs/" + name, coordList, delimiter=",")
    
def saveEdge(edgeList,fileName):
    array = np.array(edgeList)
    new_array = array.reshape([len(array),6])
    np.savetxt(str(fileName)+'.csv', new_array, delimiter=",")

def saveMap(mapping,fileName):
    file = open("./Maps/" + str(fileName) + ".txt",'w')
    for node in mapping:
        file.write(str(node) +"\n")
        
def saveDebug(string):
    file = open("./Logs/debugLog.txt",'a')
    file.write(string + "\n")
    file.close()

def saveDebugCoord(string,fileName):
    file = open("./Logs/" + fileName + ".txt",'a')
    file.write(string + "\n")
    file.close()     

def saveLists(nameFile,listCoord,nameList,coord):
    with open("./Logs/"+ nameFile, 'a') as file:
        file.write("=============================================================" + "\n")
        file.write(str(coord) + "\n")
        file.write("-----------------------" + str(nameList) + "-----------------------" + "\n")
        for coord in listCoord:
            file.write(str(coord) + "\n")
        file.write("----------------------------------------------------------------------" + "\n")
        
        