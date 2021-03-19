# -*- coding: utf-8 -*-
"""
Created on Mon Mar  8 17:46:01 2021

@author: raulf
"""

def logNeighbor(neighborhood,angles,distances,currentNode):
    print("//------------------------------------------------------------------------------------------------------------//")
    print("//---------------  Nó atual: " + str(currentNode) +" --------------------------//") 
    print("// ---------------- Coordenadas ---------------- // -------- Angulação ------- // ------- Distancia -------- //")
    for index in range(len(neighborhood)):
        print("//--- " + str(neighborhood[index])+ " ---//---  " + str(angles[index][2]) + "  ---//--- " +  str(distances[index]) + " ---//")
    print("//------------------------------------------------------------------------------------------------------------//")
    
    
def logLidar(lidarCenter,lidarLeft,lidarRight):
    print("//------------------------------------------------------------------------------------------------------------//")
    print("//---------------------------------- Log de leitura de distância insuficiente --------------------------------//")
    print("//----- Leitura da direita ---------//----- Leitura central ---------//----- Leitura Esquerda ---------//")
    for i in range(len(lidarCenter)):
        print("// --- " + str(lidarRight[i]) + " ---// --- " + str(lidarCenter[i]) + " ---// --- " + str(lidarLeft[i]) + "----//")
    print("//------------------------------------------------------------------------------------------------------------//")
    
def logNodeInfo(node):
    print("//------------------------------------------------------------------------------------------------------------//")
    print("//------------------------------------ Log de informações sobre o nó atual -----------------------------------//")
    for i in range(len(node.neighborhoodCoord)):
        print("// --- " + str(node.neighborhoodCoord[i]) + " ---// --- " + str(node.neighborhoodAngle[i]) + " ---//")
    print("//============================================================================================================//")
    print("Índice de nós visitados: " + str(node.visitedIndex))
    print("Nó atual: " + str(node.coord))
    print("Coordenadas do nó pai: " + str(node.parent.coord))
    print("Ângulo do nó pai: " + str(node.parentAngle))
    print("Distância do nó pai: " + str(node.parentDist))
    print("Coordenadas do nó atual: " + str(node.coord))
    print("//------------------------------------------------------------------------------------------------------------//")
    
    