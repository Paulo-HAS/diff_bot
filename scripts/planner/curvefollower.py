import rospy
import time
import numpy as np
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

from sensor_msgs.msg import LaserScan

goal = np.array([0.0, 0.0])    #goal (x, y)
pose = np.array([0.0, 0.0])    #config atual (x, y)
yaw = 0.0                      #yaw em rad
d_goal = []                    #vetor de distancia atual do robô com goal
scan_data = None