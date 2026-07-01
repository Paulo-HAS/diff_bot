import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
import random
import heapq



SYS_RATE = 15       # Frequência de execução
MAP_SIZE = 25       #Tamanho do mapa (n x n)

GOAL = (22, 22)     # celula de destino

class Cell:
    def __init__(self, position, g, h, parent=None):
        self.pos = position
        self.parent = parent
        self.g = g # Custo do início até este nó
        self.h = h # Custo estimado deste nó até o objetivo (Heurística)
        self.f = g + h # Custo total (g + h)

class PRM:
    def __init__(self, grid):
        self.q_start = None
        self.q_goal = None
        self.O = []
        self.C = []

        self.nodes= []

        self.grid = grid
        self.max_y = len(grid)
        self.max_x = len(grid[0])

        self.V = []
        self.E = {}
    
    def collision_free(self, q):
            x, y = q

            if x < 0 or x >= self.max_x:
                return False

            if y < 0 or y >= self.max_y:
                return False

            return self.grid[x][y] == 0
    
    def dist(self, q1, q2):

        dx = q1[0] - q2[0]
        dy = q1[1] - q2[1]

        return math.sqrt(dx*dx + dy*dy)
    
    def local_planner(self, q1, q2):

        x1, y1 = q1
        x2, y2 = q2

        dx = abs(x2 - x1)
        dy = abs(y2 - y1)

        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1

        err = dx - dy

        while True:

            if self.grid[x1][y1] != 0:
                return False

            if x1 == x2 and y1 == y2:
                break

            e2 = 2 * err

            if e2 > -dy:
                err -= dy
                x1 += sx

            if e2 < dx:
                err += dx
                y1 += sy

        return True

    def build_roadmap(self, n_samples=100, radius=5):

        self.V = []
        self.E = {}

        # ----------------------------------
        # Gerar amostras livres
        # ----------------------------------

        while len(self.V) < n_samples:

            q = (
                random.randint(0, self.max_x - 1),
                random.randint(0, self.max_y - 1)
            )

            if self.collision_free(q):

                if q not in self.V:
                    self.V.append(q)

        # ----------------------------------
        # Inicializar lista de adjacência
        # ----------------------------------

        for q in self.V:
            self.E[q] = []

        # ----------------------------------
        # Construir conexões
        # ----------------------------------

        for q in self.V:

            for q2 in self.V:

                if q == q2:
                    continue

                if self.dist(q, q2) > radius:
                    continue

                # evita aresta duplicada
                if q2 in self.E[q]:
                    continue

                if self.local_planner(q, q2):

                    self.E[q].append(q2)
                    self.E[q2].append(q)
            
    def nearest_neighbors(self, q, k):

        neighbors = []

        for node in self.V:

            if node == q:
                continue

            d = self.dist(q, node)

            neighbors.append((d, node))

        neighbors.sort(key=lambda x: x[0])

        return [node for _, node in neighbors[:k]]
    
    def connect_node(self, q, k=10):

        if q not in self.V:
            self.V.append(q)

        if q not in self.E:
            self.E[q] = []

        Nq = self.nearest_neighbors(q, k)

        for q_prime in Nq:

            if self.local_planner(q, q_prime):

                self.E[q].append(q_prime)
                self.E[q_prime].append(q)

                return True

        return False
    
    def query(self, q_init, q_goal, k=10):

        # adiciona ao roadmap
        connected_init = self.connect_node(q_init, k)
        connected_goal = self.connect_node(q_goal, k)

        if not connected_init:
            print("Falha ao conectar q_init")
            return None

        if not connected_goal:
            print("Falha ao conectar q_goal")
            return None
        
        print("Mapa com os pontos gerados")
        showmap = np.copy(self.grid)
        for x, y in self.V:
            showmap[x][y] = 2
        print(showmap)

        return self.graph_astar(q_init, q_goal)



    def graph_astar(self, start, goal):

        open_set = []

        heapq.heappush(open_set, (0, start))

        came_from = {}

        g_score = {node: float('inf') for node in self.V}
        g_score[start] = 0

        f_score = {node: float('inf') for node in self.V}
        f_score[start] = self.dist(start, goal)

        while open_set:

            _, current = heapq.heappop(open_set)

            if current == goal:

                path = []

                while current in came_from:
                    path.append(current)
                    current = came_from[current]

                path.append(start)

                return path[::-1]

            for neighbor in self.E[current]:

                tentative_g = (
                    g_score[current]
                    + self.dist(current, neighbor)
                )

                if tentative_g < g_score[neighbor]:

                    came_from[neighbor] = current

                    g_score[neighbor] = tentative_g

                    f_score[neighbor] = (
                        tentative_g
                        + self.dist(neighbor, goal)
                    )

                    heapq.heappush(
                        open_set,
                        (f_score[neighbor], neighbor)
                    )

        return None

# Node do planner
class NavNode:
    def __init__(self):
        rospy.init_node("prm", anonymous = True)
        rospy.Subscriber("/odom", Odometry, self.odomCallback)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Cria um grid 25x25 vazio (preenchido com 0)
        self.grid = [[0 for _ in range(MAP_SIZE)] for _ in range(MAP_SIZE)]

        # Adicionando alguns obstáculos (1)
        self.grid[5][5:20] = [1] * 15 # Uma parede horizontal
        for i in range(10, 20):
            self.grid[i][10] = 1 # Uma parede vertical

        self.prm = PRM(self.grid)

        self.prm.build_roadmap(n_samples=100,radius=5)

        # Estado do robô
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_received = False

        self.grid_res = 1.0     # tamanho de cada celula do grid e metros
        self.tolerance = 0.2    # Tolerância de posicionamento do robô em uma célula

        # Ganhos do controlador
        self.kp_linear = 0.5
        self.kp_angular = 1.5
        self.max_linear_vel = 0.8   # m/s
        self.max_angular_vel = 1.0  # rad/s

        self.rate = rospy.Rate(SYS_RATE)

        


    def odomCallback(self, data):
        """Atualiza a posição do robô com base na odometria."""
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y

        orientation = data.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        ])

        self.odom_received = True

    def world_to_grid(self, wx, wy):
        """Converte coordenadas em metros para índice do array."""
        gx = int(round(wx + MAP_SIZE/2))
        gy = int(round(wy + MAP_SIZE/2))

        return (gx, gy)

    def grid_to_world(self, gx, gy):
        """Converte índice do array para coordenadas em metros."""
        wx = gx - MAP_SIZE/2
        wy = gy - MAP_SIZE/2

        return (wx, wy)

    def control(self, target_x, target_y):
        """Controlador P simples para levar o robô até uma coordenada (X,Y)"""
        rospy.loginfo(f"Indo para o waypoint: ({target_x:.2f}, {target_y:.2f})")
        
        while not rospy.is_shutdown():
            # Distância e angulo até o alvo
            dx = target_x - self.x
            dy = target_y - self.y
            distance = math.hypot(dx, dy)

            if distance < self.tolerance:
                return True
            
            # Erro de angulo
            target_heading = math. atan2(dy, dx)
            heading_error = target_heading - self.yaw


            #  Normalizando o erro angular entre pi e -pi
            heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

            cmd_vel = Twist()

            # Logica de controle
            # Se o robô está muito desalinhado, gire primeiro antes de avançar muito
            if abs(heading_error) > 0.5: # Aprox 30 graus
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = self.kp_angular * heading_error
            else:
                # Proporcional puro
                cmd_vel.linear.x = self.kp_linear * distance
                cmd_vel.angular.z = self.kp_angular * heading_error
            
            # Saturação (Limita as velocidades máximas)
            cmd_vel.angular.z = max(min(cmd_vel.angular.z, self.max_angular_vel), -self.max_angular_vel)
            cmd_vel.linear.x = max(min(cmd_vel.linear.x, self.max_linear_vel), -self.max_linear_vel)

            # Envia o comando para a interface
            self.pub_cmd_vel.publish(cmd_vel)
            self.rate.sleep()


    def startPRM(self, start, goal):

        if not self.prm.collision_free(start):
            print("Start inválido")
            return None

        if not self.prm.collision_free(goal):
            print("Goal inválido")
            return None

        path = self.prm.query(start, goal, k=10)



        if path is not None:
            print("Caminho encontrado!")
        else:
            print("Nenhum caminho encontrado!")

        return path
    
    def endPRM(self, goal):
        rospy.loginfo("Roadmap percorrido. Seguindo para goal!")
        w_goal = self.grid_to_world(goal[0], goal[1])
        self.control(w_goal[0], w_goal[1])

    def stop_robot(self):
        """Envia velocidade zero."""
        cmd_vel = Twist()
        self.pub_cmd_vel.publish(cmd_vel)
            

# ==========================================
# EXECUÇÃO PRINCIPAL
# ==========================================
if __name__ == '__main__':
    try:
        nav = NavNode()

        rospy.loginfo("Aguardando os dados de odometria do robô...")
        while not nav.odom_received and not rospy.is_shutdown():
            nav.rate.sleep()

        rospy.loginfo(f"Posição inicial recebida: x={nav.x:.2f}, y={nav.y:.2f}")

        # Onde o robô está agora na perspectiva do mapa?
        start_grid = nav.world_to_grid(nav.x, nav.y)

        rospy.loginfo(f"Iniciando rota pelo roadmap de {start_grid} para {GOAL}...")
        path_grid = nav.startPRM(start_grid, GOAL)

        if path_grid is None:
            rospy.logerr("Caminho não encontrado! Verifique os obstáculos.")
        else:
            rospy.loginfo(f"Caminho encontrado com {len(path_grid)} passos. Iniciando Roadmap!")
            
            # ---------------------------------------------------------
            # Execução (Navegação waypoint a waypoint)
            # ---------------------------------------------------------
            # Ignoramos o primeiro ponto pois é onde o robô já está
            for point in path_grid[1:]:
                if rospy.is_shutdown():
                    break
                
                # Converte o ponto do grid A* para coordenadas X,Y métricas
                world_x, world_y = nav.grid_to_world(point[0], point[1])
                
                # Manda o robô para esse ponto
                nav.control(world_x, world_y)

            
            nav.endPRM(GOAL)
            rospy.loginfo("Destino alcançado!")
            nav.stop_robot()

    except rospy.ROSInterruptException:
        pass



