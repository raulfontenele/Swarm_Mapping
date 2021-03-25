# -*- coding: utf-8 -*-
"""
Created on Mon Mar  8 22:30:26 2021

@author: raulf
"""
import numpy as np
import json

def saveCoord(coordList):
    a = np.asarray(coordList)
    np.savetxt("coord.csv", a, delimiter=",")
    
def saveEdge(edgeList,fileName):

    array = np.array(edgeList)
    new_array = array.reshape([len(array),6])
    np.savetxt(str(fileName)+'.csv', new_array, delimiter=",")

def saveMap(mapping,fileName):
    file = open( str(fileName) + ".txt",'w')
    for node in mapping:
        file.write(str(node) +"\n")

        