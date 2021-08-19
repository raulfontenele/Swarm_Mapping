from enum import Enum

class Moviment(Enum):
    Foward = 1
    Hor = 2
    Antihor = 3
    Back = 4

class Order(Enum):
    Fifo = 1
    Minimum = 2
    Maximum = 3
