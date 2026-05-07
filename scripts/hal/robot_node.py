#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
import numpy as np
import time
import sys

import sys
sys.path.insert(1, '/home/paulo/movimentacao_ws/src/diff_bot/scripts/hal')

from class_pioneer import PioneerP3DX


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
            self.robot = PioneerP3DX(parameters)
            self.robot.startMission()
        except Exception as e:
            print(f"Erro ao inicializar o Pioneer: {e}")
            print("Verifique se o CoppeliaSim está em execução.")
            sys.exit(1)
    
    def drive(self, v, w):
        self.robot.setVel(vref = v, wref = w)
    
    def getVel(self):
        self.robot.getVel()

    # função para obter q = SE(2) <vou precisar publicar isso para o planner> 
    def getQ(self):
        pass

    def publishOdom(self, odom_pub):
        odom = Odometry()

        # posição
        pos = self.robot.getPos()
        odom.pose.pose.position.x = float(pos[0])
        odom.pose.pose.position.y = float(pos[1])
        odom.pose.pose.position.z = 0.0

        # orientação (quaternion)
        q = self.robot.getYawRaw()

        odom.pose.pose.orientation = Quaternion(
            x=float(q[0]),
            y=float(q[1]),
            z=float(q[2]),
            w=float(q[3])
        )

        # (opcional, mas recomendado)
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = self.robot.getVel()

        # header (MUITO IMPORTANTE em ROS)
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"

        odom_pub.publish(odom)

    def run(self, odom_pub):
        global v, w
        #Parametros do robo e do simulador
        rospy.set_param('FREQSIM', FREQ_SIM)
        
        rate = rospy.Rate(FREQ_SIM)

        while not rospy.is_shutdown():
            self.drive(v, w)
            #self.drive(2, 2)    #teste de simulação
            self.publishOdom(odom_pub)
            self.robot.step()
            rate.sleep()
            #rospy.loginfo(f"vel linear: {self.robot.getVel()[0]:.2f} | | vel angular: {self.robot.getVel()[1]:.2f}")

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
        rospy.Subscriber('/cmd_vel', Twist, velCallBack)
        odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        rospy.loginfo("Iniciando modulo de interface do Pioneer P3DX...")

        ri.run(odom_pub)
    except rospy.ROSInterruptException:
        rospy.loginfo("Encerrando modulo de interface de Pioneer P3DX...")