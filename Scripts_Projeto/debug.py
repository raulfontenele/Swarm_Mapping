# -*- coding: utf-8 -*-
"""
Created on Mon Mar  8 17:46:01 2021

@author: raulf
"""
import rich
from rich.console import Console
from rich.table import Table

def logNeighbor(neighborhood,angles,currentNode,robotId):

    table = Table(title="Neighborhood Log")

    table.add_column("Robot", justify="center", style="green", no_wrap=True)
    table.add_column("Coordinates", justify="center", style="cyan", no_wrap=True)
    table.add_column("Angles", style="magenta")

    table.add_row(str(robotId), str(currentNode), "-")
    for index in range(len(neighborhood)):
        table.add_row("Neighbor", str(neighborhood[index]), str(angles[index]))
    console = Console()
    console.print(table)

def logRobotInfo(robotObject):
    console = Console()
    console.log(robotObject)
    
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
    
    