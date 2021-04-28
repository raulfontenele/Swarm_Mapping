# -*- coding: utf-8 -*-
"""
Created on Wed Mar 24 21:24:36 2021

@author: raulf
"""

#import plotly

import plotly.graph_objects as go
import json
import matplotlib.pyplot as plt
import math

def struturedMap():
    mapping = []
    radius = 0.25

    file = open('map.txt','r')
    lines = file.readlines()

    for line in lines:
        line = line.replace("'",'"')
        struct =json.loads(line)
        mapping.append(struct)

    for row in range(len(mapping)):
        for row2 in range(len(mapping)):
            for neighbor in range(len(mapping[row2]["neighborhood"])):
                print(mapping[row]["nodeCoord"])
                print(mapping[row2]["neighborhood"][neighbor])
                diff2 = (mapping[row]["nodeCoord"][0] - mapping[row2]["neighborhood"][neighbor][0])**2 + (mapping[row]["nodeCoord"][1] - mapping[row2]["neighborhood"][neighbor][1])**2
                if math.sqrt(diff2) < radius:
                    mapping[row2]["neighborhood"][neighbor] = mapping[row]["nodeCoord"]
                    break

        

def fun2():
    file = open('map.txt','r')
    lines = file.readlines()

    for line in lines:
        line = line.replace("'",'"')
        struct =json.loads(line)
        for neighbor in struct["neighborhood"]:
            plt.plot( [struct["nodeCoord"][0],neighbor[0]],[struct["nodeCoord"][1],neighbor[1]])
            plt.scatter(struct["nodeCoord"][0],struct["nodeCoord"][1],s = 1000,marker='h')
    plt.grid()
    plt.show()

#struturedMap()
fun2()