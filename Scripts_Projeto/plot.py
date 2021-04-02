# -*- coding: utf-8 -*-
"""
Created on Wed Mar 24 21:24:36 2021

@author: raulf
"""

#import plotly


import json
import matplotlib.pyplot as plt



def fun2():
    file = open('map.txt','r')
    lines = file.readlines()

    for line in lines:
        line = line.replace("'",'"')
        struct =json.loads(line)
        for neighbor in struct["neighborhood"]:
            plt.plot( [struct["nodeCoord"][0],neighbor[0]],[struct["nodeCoord"][1],neighbor[1]])
            plt.scatter(struct["nodeCoord"][0],struct["nodeCoord"][1],s = 1000,marker='h')
    
    plt.show()

fun2()