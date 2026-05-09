#!/usr/bin/env python
import sys, os
sys.path.append("coppeliasim_zmqremoteapi/")
from coppeliasim_zmqremoteapi_client import *
import numpy as np

ROBOT = {
    'L' : 0.331, #m
    'VELSTD' : 1.0 #m/s
    
}


class PioneerP3DX:
    def __init__(self, parameters):
        self.parameters = parameters

        # inicia simulador
        self.initCoppeliaSim()
        
        self.id = parameters['robot_id']

        # tempo
        self.t = 0.0
        self.tinit = 0.0
        self.dt = 0.0

        # estados do robô
        self.v = 0
        self.v_ant = 0
        self.vref = 0
        self.w = 0
        self.w_ant = 0
        self. a = 0
        self.th = 0
        self.p = None
        self.t = self.tinit

        # filtros
        self.v_maf = MAFilter()
        self.a_maf = MAFilter()
        self.vref_maf = MAFilter()
        self.wref_maf = MAFilter()
        self.w_maf = MAFilter()
    
    # inicializa interação com o CoppeliaSim
    def initCoppeliaSim(self):
        # Cria o cliente
        RemoteAPIClient().getObject('sim').stopSimulation()
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')

        robot_name = '/PioneerP3DX_2'  #nome do robô na simulação

        self.robot = self.sim.getObject(robot_name)
        if self.robot == -1:
            print('Remote API function call returned with error code (robot): ', -1)

        # pegando os handles das rodas
        self.motorLeft = self.sim.getObject(robot_name+'/leftMotor')
        self.motorRight =self.sim.getObject(robot_name+'/rightMotor')
        print('motorLeft handle  :', self.motorLeft)
        print('motorRight handle :',self.motorRight)


    # captura os estados do robo
    def getStates(self):
        self.v_ant = self.v
        self.v, self.w = self.getVel()
        self.a = self.getAccel()
        self.th = self.getYaw()
        self.p = self.getPos()
        self.t = self.getTime() - self.tinit

        return self.p, self.v, self.a, self.th, self.w, self.t

    # Começa a missão
    def startMission(self):

        # sincroniza com o simulador
        self.client.setStepping(True)

        #começa a simulação
        self.sim.startSimulation()

        # tempo inicial
        self.tinit = self.getTime()

        # começa parado
        self.setULeft(0.0)
        self.setURight(0.0)

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
				}
				
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
    
    # direção na forma de quaternion
    def getYawRaw(self):
        while True:
            q =self.sim.getObjectQuaternion(self.robot, -1)
            if (q != -1):
                break
        
        return q

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
            yaw -= 2.0*np.pi
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
        lin, ang = self.sim.getObjectVelocity(self.robot)
        v_lin = np.linalg.norm(lin[:2])  # norma da velocidade linear no plano XY
        
        v = self.v_maf.filter(v_lin)
        
        w = self.w_maf.filter(ang[2])

        return v, w

    # retorna aceleração
    def getAccel(self):
        if self.dt == 0.0:
            return 0.0
        
        a = (self.v - self.v_ant)/self.dt

        a = self.a_maf.filter(a)

        return a
    
    # seta torque da roda esquerda
    def setULeft(self, u):
        while True:
            status = self.sim.setJointTargetVelocity(self.motorLeft, u)
            if status == 1:
                break

    # seta torque da roda direita
    def setURight(self, u):
        while True:
            status = self.sim.setJointTargetVelocity(self.motorRight, u)
            if status == 1:
                break

    # calcula a vel diferencial das rodas a partir de w
    def diffVel(self, w):
        L = ROBOT['L']
        vel_right =  (w * L) / 2.0
        vel_left  = -(w * L) / 2.0
        return vel_left, vel_right


    # seta velocidades
    def setVel(self, vref, wref):
        self.vref = self.vref_maf.filter(vref)     
        self.wref = self.wref_maf.filter(wref) 

        # vel_lin = (Vl + Vr)/2 e Vl = Vr
        l_diff, r_diff = self.diffVel(wref)
        self.setULeft(self.vref + l_diff)
        self.setURight(self.vref + r_diff)
        


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


# Filtro de média móvel
class MAFilter:
    def __init__(self, alpha=0.5):
        self.alpha = alpha
        self.alpha = np.clip(self.alpha, 0.0, 1.0)
        self.m = 0.0

    def filter(self, m):
        try:
            self.m = self.alpha*m + (1.0-self.alpha)*self.m
        except:
            print('erro...')
            self.m = m
		
        return self.m
