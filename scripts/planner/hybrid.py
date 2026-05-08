import rospy
import time
import numpy as np
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

from sensor_msgs.msg import LaserScan


SYS_RATE = 15

goal = np.array([-9.1, -2.6])  #goal (x, y)
pose = np.array([0.0, 0.0])    #config atual (x, y)
yaw = 0.0                      #yaw em rad
v = 0
w = 0

scan_data = None

# Classe da Logica
class Hybrid:
    def __init__(self):
        self.rate = rospy.Rate(SYS_RATE)

        self.a = 10.0    #Paramtero da curva
        self.d_est = 10.0    #Distancia com goal em que o potencial de atração alterna entre conico e quadratico

        self.t = 0.0
        self.dt_path = 0.02

        # Ganhos do controlador
        self.k_linear = 15.0     #linear
        self.k_angular = 20.0    #angular
        self.k_att = 2.0         #atração
        self.k_rep = 0.5        #repulsão
        
        self.x_ref = 0
        self.y_ref = 0
        self.distance_error = 0

        self.ranges = []
        self.d0 = 2.5               #Distancia minima para que o obstaculo seja considerado
        self.angle_min = 0.0
        self.angle_increment = 0.0

        self.max_linear = 10.0
        self.max_angular = 10.0


    # Gera a curva (lemniscata de Gerono) por equações paramétricas
    def generate_path(self, t):

        x_ref = self.a * np.sin(t)
        y_ref = self.a * np.sin(t) * np.cos(t)

        return x_ref, y_ref
    
    def normalize_angle(self, angle):

        while angle > np.pi:
            angle -= 2.0 * np.pi

        while angle < -np.pi:
            angle += 2.0 * np.pi

        return angle

    def attractive_force(self):
        global goal, pose

        fx = 0
        fy = 0

        if self.distance_error <= self.d_est:
            fx = self.k_att * (goal[0] - pose[0])   #atração quadrática
            fy = self.k_att * (goal[1] - pose[1])
        else:
            fx = (self.d_est * self.k_att * (goal[0] - pose[0]))/(goal[0] - pose[0])        #atração conica
            fy = (self.d_est * self.k_att * (goal[1] - pose[1]))/(goal[1] - pose[1])

        return np.array([fx, fy])
    
    #Calcula a força repulsiva
    def repulsive_force(self):
        global goal
        fx = 0.0
        fy = 0.0
        c_oi = False


        if len(self.ranges) == 0:
            return np.array([0.0, 0.0])

        for i, d in enumerate(self.ranges):
            if np.isinf(d) or np.isnan(d):
                continue

            if d < self.d0:
                c_oi = True
                angle = self.angle_min + i * self.angle_increment

                global_angle = angle + yaw

                obs_x = np.cos(global_angle)
                obs_y = np.sin(global_angle)

                # intensidade repulsiva

                force = self.k_rep * ((1.0 / self.d0) - (1.0 / d)) / (d ** 2)

                fx -= force * obs_x
                fy -= force * obs_y
        
        if c_oi == True:
            self.x_ref, self.y_ref= self.generate_path(self.t)
            goal[0] = self.x_ref
            goal[1] = self.y_ref
            self.t += self.dt_path * 0.5

        return np.array([fx, fy])
    
    def run(self):
        global v,w, goal, pose, yaw, scan_data

        # Forças
        while scan_data == None:
            pass
        self.ranges = scan_data.ranges
        self.angle_min = scan_data.angle_min
        self.angle_increment = scan_data.angle_increment

        
        
        if self.distance_error < 1.0:
            self.x_ref, self.y_ref= self.generate_path(self.t)
            goal[0] = self.x_ref
            goal[1] = self.y_ref
            self.t += self.dt_path

        U_att = self.attractive_force()
        U_rep = self.repulsive_force()
        U_total = U_att - U_rep

        error_x = goal[0] - pose[0]
        error_y = goal[1] - pose[1]
        self.distance_error = np.sqrt(error_x**2 + error_y**2)
        desired_yaw = np.arctan2(U_total[1], U_total[0])
        theta_error = self.normalize_angle(desired_yaw - yaw)

        # Controle
        linear= self.k_linear * self.distance_error
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
        print(goal)
        v = linear
        w = angular

        self.rate.sleep()
        
        




# Node do planner
class HybridNode:
    def __init__(self):
        rospy.init_node("path_follower")
        rospy.Subscriber('/hokuyo', LaserScan, self.scanCallback)
        rospy.Subscriber("/odom", Odometry, self.odomCallback)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.hy = Hybrid()

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


    def scanCallback(self, data):
        global scan_data
        scan_data = data


    def runFollower(self):
        global v, w  
        while not rospy.is_shutdown():  
            self.hy.run()
            self.publishMove(v,w)

    def publishMove(self, linear, angular):
        cmd_vel = Twist()
        cmd_vel.linear.x = linear
        cmd_vel.angular.z = angular
        self.pub_cmd_vel.publish(cmd_vel)
            

if __name__ == '__main__':
    try:
        hyn = HybridNode()
        hyn.runFollower()
    except rospy.ROSInterruptException:
        pass

