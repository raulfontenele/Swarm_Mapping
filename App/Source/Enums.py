from enum import Enum

class Moviment(Enum):
    Foward = 1
    Hor = 2
    Antihor = 3
    Back = 4

class Order(Enum):

    def __str__(self):
        return '%s' % self.value

    Fifo = 'Fifo'
    Minimum = 'Minimum'
    Maximum = 'Maximum'

class Models(Enum):
    Khepera = 1

class ExplorationMode(Enum):
    def __str__(self):
        return '%s' % self.value

    Standby = "Standby"
    Conflit = "Conflit"
    Exploring = "Exploring"
    Blocked = "Blocked"
