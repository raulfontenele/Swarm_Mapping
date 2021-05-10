# -*- coding: utf-8 -*-
"""
Created on Wed Mar 24 21:24:36 2021

@author: raulf
"""

#import plotly


import json
import matplotlib.pyplot as plt
import time
import numpy as np
import pandas as pd

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
    plt.subplot(3,1,1)
    plt.scatter(nw_coord1[:,0],nw_coord1[:,1], s = 1000, marker='h')

    nw_coord2 = np.array(coord2)
    plt.subplot(3,1,2)
    plt.scatter(nw_coord2[:,0],nw_coord2[:,1], s = 1000, marker='h')

    nw_coord3 = np.array(coord3)
    plt.subplot(3,1,3)
    plt.scatter(nw_coord3[:,0],nw_coord3[:,1], s = 1000, marker='h')

    
    
    plt.grid()
    plt.show()

def analysis():
    file = open('stepMap.txt','r')
    lines = file.readlines()
    ids  = [1,2,3]
    coordinates = []

    for line in lines:
        line = line.replace("'",'"')
        struct =json.loads(line)
        coordinates.append(struct["coord"])
    
    coordSeries = pd.Series(coordinates)
    unique = coordSeries.unique()
    print(unique)
    



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
    '''
    file = open('coord.csv','r')
    lines = file.readlines()
    plt.figure(1)
    for line in lines:
        values = line.split(',')
        plt.scatter(float(values[0]),float(values[1]), s = 1000, marker='h', c= "#0000cc")
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