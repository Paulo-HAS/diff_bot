import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion


SYS_RATE = 15       # Frequência de execução
MAP_SIZE = 25       #Tamanho do mapa (n x n)

GOAL = (22, 22)     # celula de destino

class Cell:
    def __init__(self, position, g, parent=None):
        self.pos = position
        self.parent = parent

class GVDBrushfire:
    def __init__(self, grid):
        self.q_start = None
        self.q_goal = None
        self.O = []
        self.C = []
        self.ob_count = 0 # contador dos obstaculos

        self.grid = grid

        self.max_y = len(grid)
        self.max_x = len(grid[0])

        self.brushmap, self.originmap, self.gvdmap = self.brushfire(grid)

    def origin_pointer(self, origin, x, y, directions):
        #diferencia os ponteiros dos obstáculos
        for dx, dy in directions:
            nx = x + dx
            ny = y + dy
            if origin[nx][ny] != -1:
                origin[x][y] = origin[nx][ny]
        if origin[x][y] == -1:
            self.ob_count += 1
            origin[x][y] = self.ob_count
        return origin


    def brushfire(self, grid):
        """Gera o roadmap gvd por brushfire"""
        # todas as posições são inicialmente -1
        brush = np.full((MAP_SIZE, MAP_SIZE), -1)
        origin = np.full((MAP_SIZE, MAP_SIZE), -1)
        gvd = np.full((MAP_SIZE, MAP_SIZE), 0)  # roadmap inicia zerado

        Q = []

        directions = [
            ( 0, -1),   #cima
            ( 0,  1),   #baixo
            (-1,  0),   #esquerda
            ( 1,  0),   #direita
            (-1, -1),   #cima-esquerda
            ( 1, -1),   #cima-direita
            ( 1,  1),   #baixo-direita
            (-1,  1)    #baixo-esquerda
        ]

        #inicializa as celulas 
        #obstáculos
        

        for x in range(MAP_SIZE):
            for y in range(MAP_SIZE):
                if grid[x][y] == 1:
                    brush[x][y] = 0
                    origin = self.origin_pointer(origin, x, y, directions)

                    Q.append((x,y))

        #bordas
        for x in range(MAP_SIZE):
            # borda inferior
            if brush[x][0] == -1:
                brush[x][0] = 2
                origin = self.origin_pointer(origin, x, 0, directions)
                Q.append((x,0))

            # borda superior
            if brush[x][MAP_SIZE-1] == -1:
                brush[x][MAP_SIZE-1] = 2
                origin = self.origin_pointer(origin, x, MAP_SIZE-1, directions)
                Q.append((x,MAP_SIZE-1))

        for y in range(MAP_SIZE):
            # borda esquerda
            if brush[0][y] == -1:
                brush[0][y] = 2
                origin = self.origin_pointer(origin, 0, y, directions)
                Q.append((0,y))

            # borda direita
            if brush[MAP_SIZE-1][y] == -1:
                brush[MAP_SIZE-1][y] = 2
                origin = self.origin_pointer(origin, MAP_SIZE-1, y, directions)
                Q.append((MAP_SIZE-1,y))

        
        # propagação
        while Q:
            x,y in Q.pop(0)
            for dx, dy in directions:
                nx = x + dx
                ny = y + dy
                if not (nx < self.max_x and ny < self.max_y):
                    continue

                if brush[nx][ny] != -1:
                    if origin[nx][ny] != origin[x][y]:
                        gvd[x][y] = 1       # se os brushfire de dois obstaculos colidem
                    continue                # então é uma celula do roadmap gvd

                brush[nx][ny] = brush[x][y] + 1
                origin[nx][ny] = origin[x][y]
                Q.append((nx,ny))
        print("Mapa brushfire:")
        print(brush)
        print("Roadmap:")
        print(gvd)
        return brush, origin, gvd



    def search(self, start, goal):

        st_h = abs(start[0] - goal[0]) + abs(start[1] - goal[1])
        start = Cell(start, 0, st_h)
        go_h = 0
        goal = Cell(goal, 0, go_h)

        directions = [
            ( 0, -1),   #cima
            ( 0,  1),   #baixo
            (-1,  0),   #esquerda
            ( 1,  0),   #direita
            (-1, -1),   #cima-esquerda
            ( 1, -1),   #cima-direita
            ( 1,  1),   #baixo-direita
            (-1,  1)    #baixo-esquerda
        ]

        self.O.append(start)
        while len(self.O) > 0:
            #seleciona celula de menor f
            u = min(self.O, key=lambda n: n.f)
            self.O.remove(u)

            if u.pos == goal.pos:
                path = []
                route = u
                while route is not None:
                    path.append(route.pos)
                    route = route.parent
                return path[::-1]   # Inverte a ordem, obtendo o caminho de start até goal


            if u not in self.C:
                self.C.append(u)
                for dir in directions:  #verifica a vizinhança
                    n_pos = (u.pos[0] + dir[0], u.pos[1] + dir[1])

                    # verifica se atravessou a borda
                    if n_pos[0] >(self.max_x - 1) or n_pos[0] < 0 or n_pos[1] > (self.max_y - 1) or n_pos[1] < 0:
                        continue

                    # verifica se é obstáculo
                    if self.grid[n_pos[0]][n_pos[1]] != 0:
                        continue

                    # verifica se ja está na lista fechada C
                    c_check = False
                    for c in self.C:
                        if c.pos == n_pos:
                            c_check = True
                            break
                    if c_check:
                        continue
                    
                    #checa se está em O
                    o_check = False
                    for o in self.O:
                        if o.pos == n_pos:
                            o_check = True
                            break
                    if o_check:
                        continue

                    # cria o nó com os custos e adiciona a lista aberta O
                    cost = 1.414 if dir[0] != 0 and dir[1] != 0 else 1.0
                    n_g = u.g + cost
                    n_h = abs(n_pos[0] - goal.pos[0]) + abs(n_pos[1] - goal.pos[1])     # Heurística: Distância de Manhattan
                    n_cell = Cell(n_pos, n_g, n_h, u)
                    
                    

                    self.O.append(n_cell)
                    if len(self.O) % 1000 == 0:
                        print("OPEN =", len(self.O))
                    
        return  None    # Retorna nada caso não exista um caminho

# Node do planner
class NavNode:
    def __init__(self):
        rospy.init_node("a_star", anonymous = True)
        rospy.Subscriber("/odom", Odometry, self.odomCallback)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Cria um grid 25x25 vazio (preenchido com 0)
        self.grid = [[0 for _ in range(MAP_SIZE)] for _ in range(MAP_SIZE)]

        # Adicionando alguns obstáculos (1)
        self.grid[5][5:20] = [1] * 15 # Uma parede horizontal
        for i in range(10, 20):
            self.grid[i][10] = 1 # Uma parede vertical

        self.AS = Astar(self.grid)

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


    def runAstar(self, start, goal = GOAL):
        path = self.AS.search(start, goal)
        if path is not None:
            print("Caminho encontrado!")
        else:
            print("Nenhum caminho encontrado!")

        return path

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

        rospy.loginfo(f"Calculando rota A* de {start_grid} para {GOAL}...")
        path_grid = nav.runAstar(start_grid)

        if path_grid is None:
            rospy.logerr("Caminho não encontrado! Verifique os obstáculos.")
        else:
            rospy.loginfo(f"Caminho encontrado com {len(path_grid)} passos.")
            
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

            rospy.loginfo("Destino alcançado!")
            nav.stop_robot()

    except rospy.ROSInterruptException:
        pass



