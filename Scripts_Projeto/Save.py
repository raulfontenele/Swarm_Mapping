# -*- coding: utf-8 -*-
"""
Created on Mon Mar  8 22:30:26 2021

@author: raulf
"""
import numpy as np

def saveCoord(coordList):
    a = np.asarray(coordList)
    np.savetxt("coordinates2.csv", a, delimiter=",")
    
def saveEdge(edgeList):

    array = np.array(edgeList)
    new_array = array.reshape([len(array),6])
    np.savetxt("arestas2.csv", new_array, delimiter=",")