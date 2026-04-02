#!/usr/bin/env python
import sys, os
sys.path.append("coppeliasim_zmqremoteapi/")
from coppeliasim_zmqremoteapi_client import *
import numpy as np

class PioneerP3DX:
    def __init__(self, parameters):
        self.parameters = parameters

        # inicia simulador
        self.initCoppeliaSim()
        
        # tempo
        self.t = 0.0
        self.tinit = 0.0
        self.dt = 0.0
    
    # inicializa interação com o CoppeliaSim
    def initCoppeliaSim(self):
        # Cria o cliente
        RemoteAPIClient().getObject('sim').stopSimulation()
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')

        robot_name = '/PioneerP3DX'  #nome do robô na simulação

        self.robot = self.sim.getObject(robot_name)
        if self.robot == -1:
            print('Remote API function call returned with error code (robot): ', -1)

        # pegando os handles das rodas
        self.motorLeft = self.sim.getObject(robot_name+'/leftMotor')
        self.motorRight =self.sim.getObject(robot_name+'/rightMotor')
        print('motorLeft handle  :', self.motorLeft)
        print('motorRight handle :',self.motorRight)

    # Começa a missão
    def startMission(self):

        # sincroniza com o simulador
        self.client.setStepping(True)

        #começa a simulação
        self.sim.startSimulation()

        # tempo inicial
        self.tinit = self.getTime()

        # começa parado
        self.setU(0.0)
        self.setSteer(0.0)

        # salva trajetoria
        self.saveTraj()

    # fim de missão
    def stopMission(self):
        # para o simulador
        self.sim.stopSimulation()

    def step(self):

        #passo da simulação
        self.client.step()

        # tempo anterior
        t0 = self.t

        # atualiza amostragem
        self.dt = self.t - t0

        # salva trajetoria
        self.saveTraj()
    
    # salva trajetoria do robo
    def saveTraj(self):

        # dados
        data = {	't'     : self.t, 
					'p'     : self.p, 
					'v'     : self.v,
					'a'		: self.a,
					'vref'  : self.vref,
					'th'    : self.th,
					'w'     : self.w,
					'u'     : self.u}
				
		# se ja iniciou as trajetorias
        try:
            self.traj.append(data)
		# se for a primeira vez
        except:
            self.traj = [data]

    # retorna tempo de simulação
    def getTime(self):
        t = self.sim.getSimulationTime()
        if (t != -1.0):
            return t
        
    # retorna posição do robô
    def getPos(self):
        while True:
            pos = self.sim.getObjectPosition(self.robot, -1)
            if (pos != -1):
                return np.array((pos[0], pos[1]))
    
    # retorna yaw
    def getYaw(self):
        while True:
            q =self.sim.getObjectQuaternion(self.robot, -1)
            if (q != -1):
                break
        
        yaw = self.quaternion_to_yaw(q)
        yaw -= np.pi
        while yaw < 0.0:
            yaw += 2.0*np.pi
        while yaw > 2.0*np.pi:
            yaw -= 2.0*np.piecewise
        return yaw
    
    # converte quaternion -> yaw
    def quaternion_to_yaw(self, q):
        qx, qy, qz, qw = q
		
		# normalizando quaternion
        norm = np.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
        qx /= norm
        qy /= norm
        qz /= norm
        qw /= norm       
        # calcula yaw
        yaw = np.arctan2(2 * (qx * qy + qw * qz), qw**2 + qx**2 - qy**2 - qz**2)  
        return yaw
    
    # retorna velocidades linear e angular
    def getVel(self):
        pass

    # retorna aceleração
    def getAccel(self):
        pass
    
    # seta torque da roda esquerda
    def setUleft(self, u):
        pass

    # seta torque da roda direita
    def setUleft(self, u):
        pass

    # salva
    def save(self, log):
        filename = log + ('robot%d.npz') % self.id
        data = [traj for traj in self.traj]
        np.savez(filename, data=data)

    # termina a classe
    def __del__(self):
		# fecha simulador
        self.stopMission()
		
        print ('Programa terminado!')

pnr = PioneerP3DX('jooj')