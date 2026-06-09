import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion


SYS_RATE = 15


#######################
# Discretização do ambiente
#######################



start = (0, 0)                          # posição inicial
goal = (0, 0)                           # posição objetivo

class DMap():
    def __init__(self):
        self.resolution = 0.5                        # Resolução do mapeamento
        self.grid = [[0] * 25 for _ in range(25)]    # grid do mapeamento (mapa do simulador é 25x25m centrado na origem do mundo, logo será uma matriz 25x25)
        
    #gera obstáculos no mapa
    def genOi(self): 
        for i in range(10):
            self.grid[0][j] = 1
    
    # retorna o mapa
    def getMap(self):
        return self.grid
        

class Astar():
    def __init__(self):
        self._O = []

    # busca os nós da vizinhança do nó v
    def search (self, v):
        self._O.append(v)
        u = []

        neighbors{
            (-1, 0),  # cima
            (1, 0),   # baixo
            (0, -1),  # esquerda
            (0, 1)    # direita
        }

        while self._O is not []:


        

