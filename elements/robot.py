import numpy as np
class Robot():
    def __init__(self, env, robot_id, init_position, ori):
        self.env = env
        self.robot_id = robot_id
        self.init_orientation = ori # [0, 0, 0, 0, 0, 0]
        self.init_position = init_position
        self.velocity = 0
        self.orientation = ori
        self.position = init_position
        
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
        self.allocated = False
        self.finished = False
        self.reward = 0
        self.neighbor_robots = None
        
        self.edge_passed = set()
        self.edge_not_passed = set()        
        self.reset()
        
    def reset(self):
        self.allocated = False
        self.finished = False
        self.position = self.init_position
        self.nextOri = None # next heading after turn, when velocity is larger than 0
        self.afterDis = -10.
        self.backOri = None # next back after turn, when velocity is smaller than 0
        self.backDis = 10.
        self.orientation = self.init_orientation
        self.velocity = 0.
        self.reward = 0
        self.edge_passed = set()
        self.edge_not_passed = self.env.all_edges
        self.neighbor_robots = None
        
    def assign_path(self, path):
        self.path = path
        
        self.edge = None
        self.last_vertice = None
        self.distance_last_vertice = 0        
        self.next_vertice = None
        
        for edge in self.env.edges:
            if np.array_equal(path[0][0], edge.v1.position) and np.array_equal(path[0][1], edge.v2.position):
                self.edge = edge
                break
            elif np.array_equal(path[0][0], edge.v2.position) and np.array_equal(path[0][1], edge.v1.position):
                self.edge = edge
                break            
        self.distance_next_vertice = self.edge.length
        
        tempOri = self.cal_orientation(self.edge.v1.position, self.edge.v2.position)
        if tempOri == self.orientation:
            self.next_vertice = self.edge.v2
            self.last_vertice = self.edge.v1
        else:
            self.next_vertice = self.edge.v1
            self.last_vertice = self.edge.v2
    
    def action_path(self):
        velocity = 0.2
        next_edge_pos1, next_edge_pos2 = self.path[self.passed_edge_num + 1][0], self.path[self.passed_edge_num + 1][1]        
        forward_orientation = self.cal_orientation(next_edge_pos1, next_edge_pos2)   
        return velocity, forward_orientation
    
    def step_path(self):
        if not self.finished:
            if self.passed_edge_num == len(self.path) - 1:
                velocity = 0.2
                position = self.position.copy()
                if velocity * self.env.dt - self.distance_next_vertice >= 0:
                    self.position = self.update_position(position, self.distance_next_vertice, self.orientation)
                    self.finished = True
                    self.reward = 1
                else:
                    self.position = self.update_position(position, velocity * self.env.dt, self.orientation)
                    self.distance_last_vertice += velocity * self.env.dt
                    self.distance_next_vertice -= velocity * self.env.dt 
            else:
                self.velocity, self.nextOri = self.action_path()
                self.move(self.velocity)
        else:
            self.env.robot_finihsed_set.add(self)
        
    
    def step(self, action):
        # API for control robot in low_level
        velocity, self.nextOri, self.backOri = action[0], action[1], action[2]
        self.move(velocity)
        self.neighbor_robots = self.get_neighbor_robots()
        # check_collision
        collision = self.check_collision()
        # get reward

    def get_neighbor_robots(self):
        result = []
        for robot in self.env.robots:
            # judge whether a robot is its neighbor
            # update result
            pass
        
        
    def communicate(self, neighbor_robots):
        # communicate, update self.edge_passed
        
        
        self.edge_not_passed -= self.edge_passed
        
    
    def move(self, velocity):
        self.afterDis = velocity * self.env.dt - self.distance_next_vertice
        self.backDis = self.distance_last_vertice + velocity * self.env.dt
        self.velocity = velocity
        if self.afterDis >= 0:
            self.edge_passed.add(self.edge)
            # self.edge_not_passed.remove(self.edge)            
            self.turn(self.afterDis, self.nextOri)
        elif self.backDis <= 0:
            self.turnBack(self.afterDis, self.backOri) # 后退转向
        else:
            position = self.position.copy()
            self.position = self.update_position(position, velocity * self.env.dt, self.orientation)
            self.distance_last_vertice += velocity * self.env.dt
            self.distance_next_vertice -= velocity * self.env.dt        
    
    def update_position(self, position, move_dis, orientation):
        if orientation[0] == 1:
            position[0] += move_dis
        elif orientation[1] == 1:
            position[0] -= move_dis
        elif orientation[2] == 1:
            position[1] += move_dis
        elif orientation[3] == 1:
            position[1] -= move_dis
        elif orientation[4] == 1:
            position[2] += move_dis
        elif orientation[5] == 1:
            position[2] -= move_dis
        return position
            
    def turn(self, distance, orientation):
        # data in self.next_vertice.neighbor could be choosen to turn
        self.orientation = orientation
        
        self.last_vertice = self.next_vertice
        position = self.last_vertice.position.copy()
        self.position = self.update_position(position, distance, self.orientation)
        
        self.distance_last_vertice = distance    
        
        for item in self.last_vertice.neighbors:
            if self.cal_orientation(self.last_vertice.position, item.position) == self.orientation:
                self.next_vertice = item
                break        
        for edge in self.env.edges:
            if np.array_equal(self.last_vertice.position, edge.v1.position) and np.array_equal(self.next_vertice.position, edge.v2.position):
                self.edge = edge
                break
            elif np.array_equal(self.next_vertice.position, edge.v2.position) and np.array_equal(self.last_vertice.position, edge.v1.position):
                self.edge = edge
                break
        self.distance_next_vertice = self.edge.length - self.distance_last_vertice

        self.passed_edge_num += 1
    
    def turnBack(self, distance, orientation):
        self.orientation = orientation
        
        self.next_vertice = self.last_vertice
        
        position = self.next_vertice.position.copy()
        
        backOrientation = self.reverse_orientation(self.orientation)        
        self.position = self.update_position(position, distance, backOrientation)
        
        self.distance_next_vertice = distance    
        
        for item in self.next_vertice.neighbors:
            if self.cal_orientation(item.position, self.next_vertice.position) == self.orientation:
                self.last_vertice = item
                break        
        for edge in self.env.edges:
            if np.array_equal(self.last_vertice.position, edge.v1.position) and np.array_equal(self.next_vertice.position, edge.v2.position):
                self.edge = edge
                break
            elif np.array_equal(self.next_vertice.position, edge.v2.position) and np.array_equal(self.last_vertice.position, edge.v1.position):
                self.edge = edge
                break
        self.distance_last_vertice = self.edge.length - self.distance_next_vertice
        
    def check_collision(self):
        for robot in self.env.robots:
            # cal distance
            if distance < 0.2:
                return True
        return False
    
    def cal_orientation(self, pos1, pos2):
        # pos2 相对于 pos1 的方向
        temp = pos2 - pos1
        if np.linalg.norm(temp) == 0.:
            return [0, 0, 0, 0, 0, 0]
        refer = np.array([[1., 0., 0.], [-1., 0., 0.], [0., 1., 0.], [0., -1., 0.], [0., 0., 1.], [0., 0., -1.]])
        cache = temp / np.linalg.norm(temp) - refer 
        result = np.argmin(np.linalg.norm(cache, axis=1))
        Ori = [1, 1, 1, 1, 1, 1]
        if result == 0:
            Ori = [1, 0, 0, 0, 0, 0]
        elif result == 1:
            Ori = [0, 1, 0, 0, 0, 0]
        elif result == 2:
            Ori = [0, 0, 1, 0, 0, 0]    
        elif result == 3:
            Ori = [0, 0, 0, 1, 0, 0]
        elif result == 4:
            Ori = [0, 0, 0, 0, 1, 0]
        elif result == 5:
            Ori = [0, 0, 0, 0, 0, 1]
        return Ori
        
    def reverse_orientation(self, orientation):
        Ori = [0, 0, 0, 0, 0, 0]
        if orientation[0] == 1:
            Ori = [0, 1, 0, 0, 0, 0]
        elif orientation[1] == 1:
            Ori = [1, 0, 0, 0, 0, 0]
        elif orientation[2] == 1:
            Ori = [0, 0, 0, 1, 0, 0]    
        elif orientation[3] == 1:
            Ori = [0, 0, 1, 0, 0, 0]
        elif orientation[4] == 1:
            Ori = [0, 0, 0, 0, 0, 1]
        elif orientation[5] == 1:
            Ori = [0, 0, 0, 0, 1, 0]
        return Ori
    
    
    
    