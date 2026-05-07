import rospy
import time
import numpy as np
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

from sensor_msgs.msg import LaserScan


SYS_RATE = 15


pose = np.array([0.0, 0.0])    #config atual (x, y)
yaw = 0.0                      #yaw em rad
v = 0
w = 0

# Classe da Logica
class PathFollower:
    def __init__(self):
        self.rate = rospy.Rate(SYS_RATE)

        self.a = 2.0    #Paramtero da curva

        self.t = 0.0
        self.dt_path = 0.02

        # Ganhos do controlador
        self.k_linear = 10.0     #linear
        self.k_angular = 40.0    #angular

        self.max_linear = 10.0
        self.max_angular = 10.0


    # Gera a curva (lemniscata de Gerono) por equações paramétricas
    def generate_path(self, t):

        x_ref = self.a * np.sin(t)
        y_ref = self.a * np.sin(t) * np.cos(t)

        dx_dt = self.a * np.cos(t)

        dy_dt = self.a * (
            np.cos(t)**2 - np.sin(t)**2
        )

        return x_ref, y_ref, dx_dt, dy_dt
    
    def normalize_angle(self, angle):

        while angle > np.pi:
            angle -= 2.0 * np.pi

        while angle < -np.pi:
            angle += 2.0 * np.pi

        return angle
    
    def run(self):
        global v,w, pose, yaw
        
        x_ref, y_ref, dx_dt, dy_dt = self.generate_path(self.t)

        self.t += self.dt_path

        error_x = x_ref - pose[0]
        error_y = y_ref - pose[1]
        distance_error = np.sqrt(error_x**2 + error_y**2)
        theta_ref = np.arctan2(error_y, error_x)
        theta_error = self.normalize_angle(theta_ref - yaw)

        # Controle
        linear= self.k_linear * distance_error
        angular = self.k_angular * theta_error

        # Saturação das velocidades
        linear = np.clip(
            linear,
            -self.max_linear,
            self.max_linear
        )
        angular = np.clip(
            angular,
            -self.max_angular,
            self.max_angular
        )

        v = linear
        w = angular

        self.rate.sleep()
        




# Node do planner
class PathFollowerNode:
    def __init__(self):
        rospy.init_node("path_follower")
        rospy.Subscriber("/odom", Odometry, self.odomCallback)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.pf = PathFollower()

    def odomCallback(self, data):
        global pose, yaw, d_goal
        pose[0] = data.pose.pose.position.x
        pose[1] = data.pose.pose.position.y

        orientation = data.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        ])

        #print(f'pose: ({pose[0]} {pose[1]}) /_ {yaw}')


    def runFollower(self):
        global v, w  
        while not rospy.is_shutdown():  
            self.pf.run()
            self.publishMove(v,w)

    def publishMove(self, linear, angular):
        cmd_vel = Twist()
        print(f'vel: {linear} || {angular}')
        cmd_vel.linear.x = linear
        cmd_vel.angular.z = angular
        self.pub_cmd_vel.publish(cmd_vel)
            

if __name__ == '__main__':
    try:
        pfn = PathFollowerNode()
        pfn.runFollower()
    except rospy.ROSInterruptException:
        pass

