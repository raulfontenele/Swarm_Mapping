# -*- coding: utf-8 -*-
"""
Created on Tue Feb 16 19:05:50 2021

@author: raulf
"""
import math

class AuxiliarFunctions:
    @staticmethod
    def diffAngles(initAngle,finalAngle,orientation):
        diff = finalAngle - initAngle
        if orientation == 1:
            if diff < 0:
                diff = (360 - initAngle) + finalAngle
        else:
            if diff <= 0:
                diff*=-1
            else:
                diff = 360 - (diff)
        return diff
    
    @staticmethod
    def projectCoord(orientation,initCoord,distance):
        #Inicialmente converter para o sistema natural e depois de graus para radianos
        angle = -orientation[2]+90
        angleRad = 2*math.pi*angle/360
        
        deltaX = distance*math.sin(angleRad)
        deltaY = distance*math.cos(angleRad)
        
        return [initCoord[0] + deltaX, initCoord[1] - deltaY]
    
    @staticmethod
    def oppositeAngle(angle):
        #Sempre somar 180 graus e quando passar de 360, pegar o excedente
        opAngle = angle + 180
        if opAngle >= 360:
            opAngle -= 360
        return opAngle