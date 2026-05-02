#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import numpy as np
import time
import sys

import scripts.hal.class_pioneer as cp


#########################################
##  Parâmetros de simulação
#########################################

FREQ_SIM = 15  # Frequência de atualização do sistema em Hz

#Parametros do robo
parameters = {
                'robot_id' : 1
            }

#velocidades
v = 0.0 #linear
w = 0.0 #angular


#######################################
##  Classe de atuação
#######################################

class RobotInterface:
    def __init__(self):
        

        try:
            self.robot = cp.PioneerP3DX(parameters)
            self.robot.startMission()
        except Exception as e:
            print(f"Erro ao inicializar o Pioneer: {e}")
            print("Verifique se o CoppeliaSim está em execução.")
            sys.exit(1)
    
    def drive(self, v, w):
        self.robot.setVel(vref = v, wref = w)
    
    def getVel(self):
        self.robot.getVel()

    def run(self):
        global v, w
        #Parametros do robo e do simulador
        rospy.set_param('FREQSIM', FREQ_SIM)
        
        rate = rospy.Rate(FREQ_SIM)

        while not rospy.is_shutdown():
            self.drive(v, w)
            self.robot.step()
            rate.sleep()
            rospy.loginfo(f"vel linear: {self.robot.getVel()[0]:.2f} | | vel angular: {self.robot.getVel()[1]:.2f}")

        self.robot.stopMission()

#############################################
##  node
#############################################

# callback do subscriber
def velCallBack(cmd_vel):
    global v, w
    v = cmd_vel.linear.x
    w = cmd_vel.angular.z

if __name__ == "__main__":
    ri = RobotInterface()
    rospy.loginfo("|||||||Iniciando interface do robo||||||||")
    ri.drive(0.0 , 0.0)

    try:
        # node
        rospy.init_node('diff_robot', anonymous=True)
        # subscriber
        rospy.Subscriber('diff_robot_planner', Twist, velCallBack)
        rospy.loginfo("Iniciando modulo de interface do Pioneer P3DX...")
        ri.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Encerrando modulo de interface de Pioneer P3DX...")