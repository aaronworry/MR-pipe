# 由 env 判断机器人在哪个管道内
class Robot():
    def __init__(self, env, init_position, ori):
        self.env = env
        self.init_orientation = ori # [0, 0, 0, 0, 0, 0]
        self.init_position = init_position
        self.velocity = 0
        self.orientation = ori
        self.position = init_position
        
        self.turnFlag = False
        self.nextOri = None # next heading after turn, when velocity is larger than 0
        self.afterDis = -10.
        self.backOri = None # next back after turn, when velocity is smaller than 0
        self.backDis = 10.
        
        self.edge = None
        self.last_vertice = None
        self.distance_last_vertice = 0
        self.distance_next_vertice = 0        
        self.next_vertice = None
        self.path = None
        
        self.passed_edge_num = 0

        self.color = None
        
        self.reset()
        
    def reset(self):
        self.view()
        
    def assign_path(self, path):
        self.path = path
        
    def move(self, velocity):
        self.afterDis = velocity * self.env.dt - self.distance_next_vertice
        self.backDis = self.distance_last_vertice + velocity * self.env.dt
        self.velocity = velocity
        if self.afterDis > 0:
            self.turnFlag = True
            self.turn(self.afterDis)
            self.passed_edge_num += 1
        elif self.backDis < 0:
            self.turnBack() # 后退转向
        else:
            self.position = self.update_position(self.position, velocity * self.env.dt)
            self.distance_last_vertice += velocity * self.env.dt
            self.distance_next_vertice -= velocity * self.env.dt        
    
    def update_position(self, position, move_dis):
        temp = position.copy()
        if self.orientation[0] == 1:
            temp[0] += move_dis
        elif self.orientation[1] == 1:
            temp[0] -= move_dis
        elif self.orientation[2] == 1:
            temp[1] += move_dis
        elif self.orientation[3] == 1:
            temp[1] -= move_dis
        elif self.orientation[4] == 1:
            temp[2] += move_dis
        elif self.orientation[5] == 1:
            temp[2] -= move_dis
        return temp
            
    def turn(self, distance):
        # data in self.next_vertice.neighbor could be choosen to turn
        self.orientation = self.nextOri
        self.last_vertice = self.next_vertice
        self.position = self.update_position(self.last_vertice.position, distance)
        self.distance_last_vertice = distance         
        
        # 计算方式不合理，应该用机器人所在的Edge的信息计算
        self.edge = None
        self.distance_next_vertice = np.linalg.norm(self.path[self.passed_edge_num + 1][0] - self.path[self.passed_edge_num + 1][1]) - self.distance_last_vertice
        next_vertice_position = self.path[self.passed_edge_num + 1][1]
        for item in self.env.nodes:
            if np.array_equal(next_vertice_position, item.position):
                self.next_vertice = item
                break
    
    def turnBack(self):
        pass
        
    def check_collision(self):
        pass
    
    def action(self):
        velocity = 0.2
        next_edge_pos1, next_edge_pos2 = self.path[self.passed_edge_num + 1][0], self.path[self.passed_edge_num + 1][1]
        nextOri = []
        if next_edge_pos2[0] - next_edge_pos1[0] > 0:
            nextOri = [1, 0, 0, 0, 0, 0]
        if next_edge_pos2[0] - next_edge_pos1[0] < 0:
            nextOri = [0, 1, 0, 0, 0, 0]
        if next_edge_pos2[1] - next_edge_pos1[1] > 0:
            nextOri = [0, 0, 1, 0, 0, 0]    
        if next_edge_pos2[1] - next_edge_pos1[1] < 0:
            nextOri = [1, 0, 0, 1, 0, 0]
        if next_edge_pos2[2] - next_edge_pos1[2] > 0:
            nextOri = [0, 0, 0, 0, 1, 0]
        if next_edge_pos2[2] - next_edge_pos1[2] < 0:
            nextOri = [0, 0, 0, 0, 0, 1]
        self.nextOri = nextOri    
        return velocity
    
    
    def view(self):
        self.color = 'green'