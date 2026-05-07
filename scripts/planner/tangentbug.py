import rospy
import time
import numpy as np
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

from sensor_msgs.msg import LaserScan

SYS_RATE = 15

goal = np.array([10.0, 10.0])    #goal (x, y)
pose = np.array([0.0, 0.0])    #config atual (x, y)
p_err = 0.3                    #Tolerancia de erro de posição (m)
yaw = 0.0                      #yaw em rad
d_goal = np.inf                    #vetor de distancia atual do robô com goal
d_followed = np.inf            #menor distância entre goal e o contorno que foi scaneado   
d_reach = np.inf               #menor distancia entre goal e o obstáculo dentro do campo de visão do robô
last_motion_ang = 0            # angulo do ultimo movimento executado
wstar = 0.8                    #distancia de segurança para formar W*
Vmax = 30.0                #Velocidade máxima do robo
tan_list = []
bound_pos = []

scan_data = None


class TangentBug:
    def __init__(self):
        self.v = 0
        self.w = 0
        self.Kp = 0.3               # ganho prporcional do controlador de trajetoria
        self.state = "mtg" # mtg = motion-to-goal || bf = boundary-following

    # Obtem a direção do q goal no frame do referencial do mundo
    def direct2goal(self):
        global pose, yaw, goal
    
        ang = np.arctan2(goal[1] - pose[1], goal[0] - pose[0]) - yaw
        ang = (ang + np.pi) % (2 * np.pi) - np.pi # obtem angulo no range de -pi a pi
    
        return ang

    # checa por continuidades no scan
    def check_cont(self):
        global scan_data
        max_dist = scan_data.range_max
        ranges = scan_data.ranges
        # recebe os indices do scan que possuem range menor que o máximo (superfície)
        cont_i= np.nonzero(np.array(ranges) < max_dist)[0]
        # recebe os indices onde começam e terminam uma continuidade
        lim_sup = np.array([x for i, x in enumerate(cont_i) 
                        if (x + 1 != cont_i[(i + 1) % len(cont_i)])])
        lim_inf = np.array([x for i, x in enumerate(cont_i) 
                        if (x - 1 != cont_i[(i - 1) % len(cont_i)])])
        # Organiza os limites em tuplas
        cont_lims = [x for x in zip(lim_inf, lim_sup) if x[0] != x[1]]
        return cont_lims
    
    # retorna as coordenadas cartesianas da leitura de um feixe do scan no frame do mundo
    def range2coord(self, idx):
        global scan_data, pose, yaw
        ranges = scan_data.ranges
        scan_step = scan_data.angle_increment
        ang_min = scan_data.angle_min

        T0r = np.array([        # Tranformação homogênea do robô em relação ao frame do mundo
                        [np.cos(yaw), -np.sin(yaw), 0, pose[0]],
                        [np.sin(yaw), np.cos(yaw), 0, pose[1]],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]
                    ])
        rangex_r = ranges[idx] * np.cos(scan_step * idx + ang_min)
        rangey_r = ranges[idx] * np.sin(scan_step * idx + ang_min)
        range_r = np.array([rangex_r, rangey_r, 1, 1]).reshape(-1, 1)
        range_w = (T0r @ range_r).ravel()
        
        return range_w[:2]

    # retorna as coordenadas cartesianas das continuidades encontradas em relação ao mundo
    def cont_coord(self, conts):
        global scan_data
        ranges = scan_data.ranges
        coord = np.empty((len(conts), 2))
        for i, x in enumerate(conts):
            coord[i, :] = self.range2coord(int(x))

        return coord

    # Checa por obstáculos entre o robô e q_goal
    def obstacle_blocking(self):
        global scan_data, pose, d_goal
        if scan_data is None:
            return False
        scan_step = scan_data.angle_increment
        ang_min = scan_data.angle_min

        d2g = self.direct2goal()

        conts = self.check_cont()
        reg_num = None
        blocking = False
        
        # itera entre regiões de continuidade encontradas e verifica se interceptam a direção para goal
        for i, region in enumerate(conts):
            lim_inf = scan_step * region[0] + ang_min
            lim_sup = scan_step * region[1] + ang_min
            if lim_inf <= d2g <= lim_sup:
                oi_mat = self.cont_coord(list(region))
                oi_inf_dist = np.linalg.norm(oi_mat[0] - np.array([pose[0], pose[1]]))
                oi_sup_dist = np.linalg.norm(oi_mat[1] - np.array([pose[0], pose[1]]))
                if oi_inf_dist < d_goal or oi_sup_dist < d_goal:
                    blocking = True
                    reg_num = i
                    break

        return blocking, reg_num
    
    #retorna a uma lista de coordenadas catesianas representando cada matriz 
    def obs_coord(self, conts_list):
        global scan_data, pose
        ranges = scan_data.ranges
        obs_mat = np.empty((len(conts_list), 2))
        for i, x in enumerate(conts_list):
            obs_mat[i, :] = self.range2coord(ranges, int(x))

        return obs_mat

    # retorna o vetor da tangente baseado na norma euclidiana
    def tanvect(self, region):
        global last_motion_ang
        reg_i = []

        if isinstance(region, tuple):
            region = [region]
        for lims in region:
            lim_inf, lim_sup = lims
            idxs = np.unique(np.linspace(lim_inf, lim_sup, -(-3*(lim_sup - lim_inf) // 4) + 1, dtype=int))
            reg_idx = np.r_[np.array(reg_idx), idxs]

        oi_mat = self.obs_coord(reg_idx)
        pos_vec = np.array([pose[0], pose[1]])
        norm_mat = np.linalg.norm(pos_vec - oi_mat, axis=1) ** 2
        min_idx = np.argmin(norm_mat)
        min_dist = norm_mat[min_idx]
        h = 0.1
        sum_, div_ = np.array([0.0, 0.0]), 0
        
        for i, a in enumerate(norm_mat):
            temp = np.exp((min_dist - a) / (2 * h ** 2))
            sum_ += temp * (pos_vec - oi_mat[i, :])
            div_ += temp
        
        rot90 = np.array([
                [np.cos(last_motion_ang), -np.sin(last_motion_ang)],
                [np.sin(last_motion_ang), np.cos(last_motion_ang)]
            ])
        D = sum_ / div_
        tangent = (rot90 @ D.reshape(-1, 1)).ravel()
        closest_point = pos_vec - D
        
        return tangent, closest_point

    # estbelece uma distancia de segurança com os obstáculos
    def safety_distance(self, oi_coord, tangent_vec):
        global pose
        vec_r2obs = oi_coord - [pose[0], pose[1]]
        obs_dist = np.linalg.norm(vec_r2obs)
        if obs_dist == 0:
            obs_dist = 1e-3
        
        oi_norm = vec_r2obs / obs_dist
        if obs_dist > 1.3*wstar:
            alpha = 1
            beta = 1
        else:
            alpha = 3
            beta = 1

        oi_safe = beta * (vec_r2obs - (wstar * oi_norm)) + alpha * tangent_vec

        return oi_safe

    # retorna as corrdenadas do melhor obstaculo a seguir quando em mtg
    def select_obs(self, conts):
        global scan_data, pose, goal, d_reach, d_followed
        ranges = scan_data.ranges
        pos_vec = np.array([pose[0], pose[1]])
        goal_vec = np.array([goal[0], goal[1]])
        
        if isinstance(conts, tuple):
            conts_list = list(conts)
        else:
            conts_list = [x for t in conts for x in t]

        oi_mat = self.obs_coord(conts_list)
        dist_rob_obs = np.linalg.norm((pos_vec - oi_mat), axis=1)
        dist_obs_goal = np.linalg.norm((goal_vec - oi_mat), axis=1)
        heuristic = dist_rob_obs + dist_obs_goal
        
        if self.state == 1:
            tangent, closest_point = self.tanvect(conts)
            safe_oi = self.safety_distance(closest_point, tangent)
            d_reach = np.linalg.norm(goal_vec - closest_point)
            oi2follow_coord = safe_oi
            
        else:
            tangent, closest_point = self.tanvect(conts)
            safe_oi = self.safety_distance(closest_point, tangent)
            d_reach = np.linalg.norm(goal_vec - closest_point)
            oi2follow_coord = safe_oi

        if d_reach <= d_followed:
            d_followed = d_reach

        return oi2follow_coord

    #Controlador de trajetoria
    def traj_controller(self, vx=0, vy=0):
        global goal, pose, yaw, Vmax
        d = 0.1
        u1 = vx + self.Kp * (goal[0] - pose[0])
        u2 = vy + self.Kp * (goal[1] - pose[1])
        Vtot = np.sqrt(u1**2 + u2**2)
        if (Vtot > Vmax):
            u1 = u1 * Vmax / Vtot
            u2 = u2 * Vmax / Vtot
        # feddback linearization
        A = [
            [np.cos(yaw), -d * np.sin(yaw)],
            [np.sin(yaw), d * np.cos(yaw)]
            ]
        vw = np.linalg.inv(A) @ [[u1], [u2]]
        v = float(vw[0])
        w = float(vw[1])
        
        return v, w

    #Controlador de trajetoria por velocidade
    def traj_controller2(self, vx, vy):
        global  yaw, Vmax
        d = 0.1
        u1 = vx
        u2 = vy
        
        Vtot = np.sqrt(u1**2 + u2**2)
        if (Vtot > Vmax):
            u1 = u1 * Vmax / Vtot
            u2 = u2 * Vmax / Vtot
        
        A = [
            [np.cos(yaw), -d * np.sin(yaw)],
            [np.sin(yaw), d * np.cos(yaw)]
            ]
        vw = np.linalg.inv(A) @ [[u1], [u2]]
        v = float(vw[0])
        w = float(vw[1])

        return v, w

    # comportamento motion-to-goal
    def motion2goal(self):

        global goal, scan_data, d_followed, last_motion_ang, tan_list
        ranges = scan_data.ranges
        conts = self.check_cont()
        blocking, reg_num = self.obstacle_blocking()
        
        if blocking:
            d_reach_old = d_reach
            oi = self.select_obs(conts[reg_num])
            oi_safe = oi
            v, w = self.traj_controller2(oi_safe[0], oi_safe[1])

            if np.linalg.norm(([goal[0], goal[1]] - oi_safe)) > d_goal:
                d_followed = d_reach_old
                tan_list = []
                self.state = 'bf'
        else:
            v, w = self.traj_controller()
            oi_safe = np.array([goal[0], goal[1]])
        
        last_motion_vec = [pose[0], pose[1]] - oi_safe
        norm_vec = last_motion_vec / np.linalg.norm(last_motion_vec)
        dot_prod = np.dot(norm_vec, np.array([1, 0]))
        if np.sign(np.arctan(dot_prod) + 2*np.pi) == 1:
            last_motion_ang = np.pi/2
        else:
            last_motion_ang = -np.pi/2
        return v, w

    #ccomportamento boundary-following
    def boundary_following(self):
        global pose, p_err, goal, d_followed, tan_list, bound_pos
        cont_lims = self.check_cont()

        if not cont_lims:
            self.state = 'mtg'
            return 0, 0
        pos_vec = np.array([pose[0], pose[1]])
        
        if bound_pos:
            bound_dists = np.linalg.norm(pos_vec - np.array(bound_pos), axis=1)
            
            if not check_loop and np.max(bound_dists) > p_err:
                check_loop = 1
            elif check_loop and any(bound_dists[:-15] <= p_err):
                self.state = 'goal'
                return 0, 0

        closest_oi = self.select_obs(cont_lims)
        oi = closest_oi
        bound_pos.append([pose[0], pose[1]])
        x_new = oi[0]
        y_new = oi[1]
        v, w = self.traj_controller2(x_new, y_new)
        
        if d_reach <= d_followed - p_err:
            tan_list = []
            self.state = 'mtg'
            bound_pos = np.array([pose[0], pose[1]])

        return v, w



    ###############################
    #Roda a o tangent bug
    def run(self):
        global pose, goal, p_err, scan_data

        rate = rospy.Rate(SYS_RATE)

        if scan_data == None:
            return


        if np.linalg.norm([pose[0] - goal[0], pose[1] - goal[1]]) < p_err and self.state != 'goal':
            self.state = 'goal'

        if self.state == 'mtg':
            self.v, self.w = self.motion2goal()
        elif self.state == 'bf':
            self.v, self.w = self.boundary_following()
        elif self.state == 'goal':
            print('_\|/_GOAL REACHED!!!!')
            self.v = 0
            self.w = 0
            com = input('Enter a new goal value (X Y) to keep going or pre Ctrl+C to Exit: ')
            goal = com
            print(f'New goal: {goal}')
            d_goal = []              
            d_followed = np.inf 
            d_reach = np.inf  
            bound_pos = []
            self.state = 'mtg'
        else:
            print('Invalid State|||||||||')
            
        rate.sleep()


    def getPlanVel(self):
        return self.v, self.w 


# Node do planner
class TangentBugNode:
    def __init__(self):
        rospy.init_node("tangent_bug")
        rospy.Subscriber('/hokuyo', LaserScan, self.scanCallback)
        rospy.Subscriber("/odom", Odometry, self.odomCallback)
        self.pub_cmd_vel = rospy.Publisher('tangent_bug/cmd_vel', Twist, queue_size=10)
        
        self.tb = TangentBug()

    def odomCallback(self, data):
        global pose, yaw
        pose[0] = data.pose.pose.position.x
        pose[1] = data.pose.pose.position.y

        orientation = data.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        ])

        print(f'pose: ({pose[0]} {pose[1]})/_ {yaw}')
        d_goal = np.linalg.norm(pose - goal)



    def scanCallback(self, data):
        global scan_data
        scan_data = data

    def runBug(self):  
        while not rospy.is_shutdown():  
            self.tb.run()
            v, w = self.tb.getPlanVel()
            self.publishMove(v,w)

    def publishMove(self, linear, angular):
        cmd_vel = Twist()
        #print(f'vel: {linear} || {angular}')
        cmd_vel.linear.x = linear
        cmd_vel.angular.z = angular
        self.pub_cmd_vel.publish(cmd_vel)
            

if __name__ == '__main__':
    try:
        tbn = TangentBugNode()

        goal[0] = 10.0
        goal[1] = 10.0
        tbn.runBug()
    except rospy.ROSInterruptException:
        pass

