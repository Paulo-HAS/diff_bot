#!/usr/bin/env python
import sys, os
sys.path.append("coppeliasim_zmqremoteapi/")
from coppeliasim_zmqremoteapi_client import *

class PioneerP3DX:
    def __init__(self, parameters):
        self.parameters = parameters

        # inicia simulador
        self.initCoppeliaSim()
        
        # tempo
        self.t = 0.0

        self.tinit = 0.0

        # tempo de amostragem
        self.dt = 0.0
    
    def initCoppeliaSim(self):
        # Cria o cliente
        RemoteAPIClient().getObject('sim').stopSimulation()
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')

        robot_name = '/PioneerP3DX'  #nome do robô na simulação

        self.robot = self.sim.getObject(robot_name)
        if self.robot == -1:
            print('Remote API function call returned with error code (robot): ', -1)

        # pegando os hanles das rodas
        self.motorLeft = self.sim.getObject(robot_name+'/leftMotor')
        self.motorRight =self.sim.getObject(robot_name+'/rightMotor')
        print('motorLeft handle :', self.motorLeft)
        print('motorRight handle :',self.motorRight)

    

pnr = PioneerP3DX('jooj')