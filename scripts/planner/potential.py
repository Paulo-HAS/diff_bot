import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion



SYS_RATE = 15

goal = np.array([-9.1, -2.6])      #goal (x, y)
pose = np.array([0.0, 0.0])        #config atual (x, y)
p_err = 0.3                        #Tolerancia de erro de posição (m)
yaw = 0.0                          #yaw em rad
d_goal = np.inf                    #vetor de distancia atual do robô com goal

scan_data = None

class PotentialBot:
    def __init__(self):
        self.rate = rospy.Rate(SYS_RATE)

        self.d_est = 10.0    #Distancia com goal em que o potencial de atração alterna entre conico e quadratico

        self.t = 0.0
        self.dt_path = 0.02

        # Ganhos do controlador
        self.k_linear = 1.0     #linear
        self.k_angular = 20.0    #angular
        self.k_att = 1.0         #atração
        self.k_rep = 5.0         #repulsão
        
        self.v = 0.0
        self.w = 0.0

        self.max_linear = 20.0
        self.max_angular = 20.0

        self.ranges = []
        self.d0 = 2.5               #Distancia minima para que o obstaculo seja considerado
        self.angle_min = 0.0
        self.angle_increment = 0.0

    #Calcula a força atrativa
    def attractive_force(self):
        global goal, pose, d_goal

        fx = 0
        fy = 0

        if d_goal <= self.d_est:
            fx = self.k_att * (goal[0] - pose[0])   #atração quadrática
            fy = self.k_att * (goal[1] - pose[1])
        else:
            fx = (self.d_est * self.k_att * (goal[0] - pose[0]))/(goal[0] - pose[0])        #atração conica
            fy = (self.d_est * self.k_att * (goal[1] - pose[1]))/(goal[1] - pose[1])

        return np.array([fx, fy])

    #Calcula a força repulsiva
    def repulsive_force(self):

        fx = 0.0
        fy = 0.0


        if len(self.ranges) == 0:
            return np.array([0.0, 0.0])

        for i, d in enumerate(self.ranges):
            if np.isinf(d) or np.isnan(d):
                continue

            if d < self.d0:

                angle = self.angle_min + i * self.angle_increment

                # direção do obstáculo no frame do robô
                obs_x = np.cos(angle)
                obs_y = np.sin(angle)

                # intensidade repulsiva

                force = self.k_rep * ((1.0 / self.d0) - (1.0 / d)) / (d ** 2)

                fx -= force * obs_x
                fy -= force * obs_y

        return np.array([fx, fy])
    
    def run(self):
        global yaw, d_goal, p_err, scan_data
        # Forças
        while scan_data == None:
            pass
        self.ranges = scan_data.ranges
        self.angle_min = scan_data.angle_min
        self.angle_increment = scan_data.angle_increment


        U_att = self.attractive_force()
        U_rep = self.repulsive_force()
        print(U_rep)
        U_total = U_att + U_rep

        desired_yaw = np.arctan2(U_total[1], U_total[0])

        # Erro angular
        error_theta = desired_yaw - yaw

        # normaliza ângulo
        error_theta = np.arctan2(
            np.sin(error_theta),
            np.cos(error_theta)
        )


        if d_goal < p_err:
            print('_\|/_GOAL REACHED!!!!')
            self.v = 0
            self.w = 0
            
        else:
            #Controle proporcional
            self.v = self.k_linear * d_goal
            self.w = self.k_angular * error_theta
            # Saturação das velocidades
            self.v = np.clip(
                self.v,
                -self.max_linear,
                self.max_linear
            )
            self.w = np.clip(
                self.w,
                -self.max_angular,
                self.max_angular
            )

        self.rate.sleep()

    def getPlanVel(self):
        #print(f'vel:   {self.v} || {self.w}')
        return self.v, self.w
        




# Node do planner
class PotentialBotNode:
    def __init__(self):
        rospy.init_node("potential_bot")
        rospy.Subscriber('/hokuyo', LaserScan, self.scanCallback)
        rospy.Subscriber("/odom", Odometry, self.odomCallback)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.pb = PotentialBot()

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

        #print(f'pose: ({pose[0]} {pose[1]})/_ {yaw}')
        d_goal = np.linalg.norm(pose - goal)



    def scanCallback(self, data):
        global scan_data
        scan_data = data

    def runPotential(self):  
        while not rospy.is_shutdown():  
            self.pb.run()
            v, w = self.pb.getPlanVel()
            self.publishMove(v,w)

    def publishMove(self, linear, angular):
        cmd_vel = Twist()
        #print(f'vel: {linear} || {angular}')
        cmd_vel.linear.x = linear
        cmd_vel.angular.z = angular
        self.pub_cmd_vel.publish(cmd_vel)
            

if __name__ == '__main__':
    try:
        pbn = PotentialBotNode()
        pbn.runPotential()
    except rospy.ROSInterruptException:
        pass

