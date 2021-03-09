# -*- coding: utf-8 -*-
"""
Created on Mon Mar  8 17:46:01 2021

@author: raulf
"""

def logNeighbor(neighborhood,angles,distances,currentNode):
    print("//---------------------------------------------------------------------------------------------//")
    print("//-------  Nó atual: " + str(currentNode) +" --------------------------//") 
    print("// ---------------- Coordenadas ---------------- // -------- Angulação ------- // ------- Distancia -------- //")
    for index in range(len(neighborhood)):
        print("//--- " + str(neighborhood[index])+ " ---//---  " + str(angles[index][2]) + "  ---//--- " +  str(distances[index]) + " ---//")
    print("//---------------------------------------------------------------------------------------------//")