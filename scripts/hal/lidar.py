#!/usr/bin/env python
import sys, os
sys.path.append("coppeliasim_zmqremoteapi/")
from coppeliasim_zmqremoteapi_client import *
import numpy as np

from sensor_msgs.msg import LaserScan


#####################
# Parametros do Laser

MIN_RANGE = 0   # alcance mínimo
MAX_RANGE = 0   # alcance máximo
MIN_ANG = 0     # angulo minimo
MAX_ANG = 0     # angulo maximo

class Lidar:
    def __init__(self):
        self.client = RemoteAPIClient()
        self.sim = self.client.require('sim')
        self.robot_name = '/PioneerP3DX'  #nome do robô na simulação
        self.sensor = self.sim.getObject(self.robot_name+'/VelodyneVPL16')

        



