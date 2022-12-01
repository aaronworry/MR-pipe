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
        self.path_passed = []
        
        self.path_for_obstacle_avoidance = []
        self.conflict_resolution_flag = False
        self.conflict_resolution_forward_flag = False
        self.prior_robots = set()
        self.collision_path_id = None
        self.hold_position_id = None   # 机器人是否站在避障点上

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
        self.path_passed = []
        
        self.path_for_obstacle_avoidance = []
        self.conflict_resolution_flag = False
        self.conflict_resolution_forward_flag = False
        self.prior_robots = set()
        self.collision_path_id = None
        self.hold_position_id = None
        
    def assign_path(self, path):
        self.path = path
        
        self.edge = None
        self.last_vertice = None
        self.distance_last_vertice = 0        
        self.next_vertice = None
        
        for edge in self.env.edges:
            if np.array_equal(self.env.graph.get_position_of_node(path[0]), edge.v1.position) and np.array_equal(self.env.graph.get_position_of_node(path[1]), edge.v2.position):
                self.edge = edge
                break
            elif np.array_equal(self.env.graph.get_position_of_node(path[0]), edge.v2.position) and np.array_equal(self.env.graph.get_position_of_node(path[1]), edge.v1.position):
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
        self.path_passed.append(self.path[0])
        self.path = self.path[1:]
    
    def action_path(self, path):
        velocity = 0.2
        next_edge1, next_edge2 = path[0], path[1]
        next_edge_pos1, next_edge_pos2 = self.env.graph.get_position_of_node(next_edge1), self.env.graph.get_position_of_node(next_edge2)       
        forward_orientation = self.cal_orientation(next_edge_pos1, next_edge_pos2)   
        return velocity, forward_orientation
    
    def step_path(self):
        self.hold_position_id = None
        if not self.finished:
            if len(self.path) == 1:
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
                self.velocity, self.nextOri = self.action_path(self.path)
                self.move(self.velocity)
        else:
            self.env.robot_finihsed_set.add(self)
        
    def step_avoidance_path(self):
        if len(self.path_for_obstacle_avoidance) == 0:
            if len(self.prior_robots) == 0:
                self.conflict_resolution_flag = False
                self.env.collision_finished_robots.add(self)
                self.collision_path_id = None
        elif len(self.path_for_obstacle_avoidance) == 1:
            velocity = 0.2
            position = self.position.copy()
            if velocity * self.env.dt - self.distance_next_vertice >= 0:
                self.position = self.next_vertice.position
                self.hold_position_id = self.collision_path_id
                self.path_for_obstacle_avoidance = []
                # self.path = [self.path_for_obstacle_avoidance[0]] + self.path
                # self.path_passed = self.path_passed[:-1]
                self.orientation = self.reverse_orientation(self.orientation)
                self.last_vertice, self.next_vertice = self.next_vertice, self.last_vertice                
                self.distance_next_vertice = self.distance_last_vertice + self.distance_next_vertice
                self.distance_last_vertice = 0    
            else:
                self.position = self.update_position(position, velocity * self.env.dt, self.orientation)
                self.distance_last_vertice += velocity * self.env.dt
                self.distance_next_vertice -= velocity * self.env.dt 
        else:
            self.velocity, self.nextOri = self.action_path(self.path_for_obstacle_avoidance)
            self.afterDis = velocity * self.env.dt - self.distance_next_vertice
            self.velocity = velocity
            if self.afterDis >= 0:         
                self.path_passed = self.path_passed[:-1]
                self.path_for_obstacle_avoidance = self.path_for_obstacle_avoidance[1:]
                self.path = [self.path_for_obstacle_avoidance[0]] + self.path
                
                self.orientation = self.nextOri
                
                self.last_vertice = self.next_vertice
                position = self.last_vertice.position.copy()
                self.position = self.update_position(position, self.afterDis, self.orientation)
                
                self.distance_last_vertice = self.afterDis    
                
                for item in self.last_vertice.neighbors:
                    if self.cal_orientation(self.last_vertice.position, item.position) == self.orientation:
                        self.next_vertice = item
                        break        
                for edge in self.env.edges:
                    if np.array_equal(self.last_vertice.position, edge.v1.position) and np.array_equal(self.next_vertice.position, edge.v2.position):
                        self.edge = edge
                        break
                    elif np.array_equal(self.last_vertice.position, edge.v2.position) and np.array_equal(self.next_vertice.position, edge.v1.position):
                        self.edge = edge
                        break
                self.distance_next_vertice = self.edge.length - self.distance_last_vertice
            else:
                position = self.position.copy()
                self.position = self.update_position(position, velocity * self.env.dt, self.orientation)
                self.distance_last_vertice += velocity * self.env.dt
                self.distance_next_vertice -= velocity * self.env.dt

    def step_avoidance_forward_path(self):
        if self.collision_path_id == self.path[0]:
            velocity = 0.2
            position = self.position.copy()
            if velocity * self.env.dt - self.distance_next_vertice >= 0:
                self.position = self.update_position(position, self.distance_next_vertice, self.orientation)
                self.hold_position_id = self.collision_path_id
                self.edge_passed.add(self.edge)
                self.path_passed.append(self.path[0])
                self.path = self.path[1:]
            else:
                self.position = self.update_position(position, velocity * self.env.dt, self.orientation)
                self.distance_last_vertice += velocity * self.env.dt
                self.distance_next_vertice -= velocity * self.env.dt
        else:
            if len(self.prior_robots) == 0:
                self.conflict_resolution_forward_flag = False
                self.env.collision_finished_robots.add(self)
                self.collision_path_id = None
        
    def collision_avoidance(self):
        if self.conflict_resolution_flag or self.conflict_resolution_forward_flag:
            print(1)
        else:
            neighbor_cache = self.get_neighbor_robots()
            if len(neighbor_cache) >= 1:
                for robot in neighbor_cache:
                    self.judge_collision_update_prior(robot)
                if self.conflict_resolution_flag:
                    """写在哪儿比较好？"""
                    # 更新 self.path_for_obstacle_avoidance, 改变方向
                    temp = self.path_passed.copy()
                    temp.reverse()
                    for item in temp:
                        self.path_for_obstacle_avoidance.append(item)
                        if item in self.env.graph.degree134:                            
                            break
                    self.reverse_state()

                    
    
    def judge_collision_update_prior(self, robot):
        # 查看自己是否在其他机器人的prior中，如果在则跳过
        # 如果不在，判断自己是否需要参与避障
        if self not in robot.prior_robots:
            # 判断是否需要主动避开别人：  别人是否在避障
            if robot.conflict_resolution_forward_flag or robot.conflict_resolution_flag:
                if self.path[0] == robot.collision_path_id:  # 自己的下一个点和参与避障的机器人的目标相同
                    # 向前移动避障
                    self.collision_path_id = robot.collision_path_id
                    if robot.hold_position_id:
                        robot.prior_robots.add(self)
                    elif robot.distance_next_vertice >= self.distance_next_vertice:
                        self.prior_robots.add(robot)
                        self.conflict_resolution_forward_flag = True
                    else:
                        robot.prior_robots.add(self)
                elif robot.collision_path_id in self.path_passed:
                    # 向后移动避障
                    self.collision_path_id = robot.collision_path_id
                    self.conflict_resolution_flag = True
                    self.prior_robots.add(robot)
                else:
                    # 目前不需要避障
                    print(2)
            else:
                # 对方也是自由机器人, 需要更改对方
                # 如果需要主动避开别人，将对方加入到自己的prior中，得到避障点的path_id                
                # 在同一个管道中：
                if self.edge == robot.edge:
                    if self.orientation == robot.orientation:
                        print(3)
                    else:
                        if len(self.path) <= len(robot.path):
                            self.conflict_resolution_flag = True
                            self.prior_robots.add(robot)
                            
                            if self.collision_path_id == None:
                                temp = self.path_passed.copy()
                                temp.reverse()
                                for item in temp:
                                    if item in self.env.graph.degree134:
                                        self.collision_path_id = item
                                        break
                            robot.collision_path_id = self.collision_path_id    
                        else:
                            print(4)
                else:
                    # 两个机器人对在一起
                    if self.path[0] == robot.path[0]:
                        # 节点的入度为 2 时：
                        if self.path[0] not in self.env.graph.degree134:
                            if len(self.path) <= len(robot.path):
                                self.conflict_resolution_flag = True
                                self.prior_robots.add(robot)
                                
                                if self.collision_path_id == None:
                                    temp = self.path_passed.copy()
                                    temp.reverse()
                                    for item in temp:
                                        if item in self.env.graph.degree134:
                                            self.collision_path_id = item
                                            break
                                robot.collision_path_id = self.collision_path_id    
                            else:
                                print(5)
                        # 度为 3 4 时
                        else:
                            if np.array_equal(self.env.graph.get_position_of_node(self.path[1]), self.last_vertice.position):
                                print(6)
                            elif self.distance_next_vertice <= robot.distance_next_vertice and np.array_equal(self.env.graph.get_position_of_node(robot.path[1]), self.last_vertice.position):
                                self.conflict_resolution_forward_flag = True
                                self.prior_robots.add(robot)
                                self.collision_path_id = self.path[0]
                                robot.collision_path_id = self.collision_path_id
                            elif self.distance_next_vertice >= robot.distance_next_vertice and self.path[1] == robot.path[1]:
                                self.conflict_resolution_forward_flag = True
                                self.prior_robots.add(robot)
                                self.collision_path_id = self.path[0]
                                robot.collision_path_id = self.collision_path_id
                            else:
                                print(7)
                    else:
                        print(8)
            
    
    
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
            distance = np.linalg.norm(self.position - robot.position)
            if distance > 0.03 and distance <= 0.5:
                result.append(robot)
        return result
        
        
    def communicate(self, neighbor_robots):
        # communicate, update self.edge_passed
        
        
        self.edge_not_passed -= self.edge_passed
        
    
    def move(self, velocity):
        self.afterDis = velocity * self.env.dt - self.distance_next_vertice
        self.backDis = self.distance_last_vertice + velocity * self.env.dt
        self.velocity = velocity
        if self.afterDis >= 0:         
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
        
        self.edge_passed.add(self.edge)
        self.path_passed.append(self.path[0])
        if self.collision_path_id == self.path[0]:
            self.env.collision_finished_robots.add(self)
        self.path = self.path[1:]
        
        
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
            elif np.array_equal(self.last_vertice.position, edge.v2.position) and np.array_equal(self.next_vertice.position, edge.v1.position):
                self.edge = edge
                break
        self.distance_next_vertice = self.edge.length - self.distance_last_vertice
        
        
    
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
            elif np.array_equal(self.last_vertice.position, edge.v2.position) and np.array_equal(self.next_vertice.position, edge.v1.position):
                self.edge = edge
                break
        self.distance_last_vertice = self.edge.length - self.distance_next_vertice
        
    def check_collision(self):
        for robot in self.env.robots:
            distance = np.linalg.norm(self.position - robot.position)
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
        
    def reverse_state(self):        
        self.orientation = self.reverse_orientation(self.orientation)
        self.last_vertice, self.next_vertice = self.next_vertice, self.last_vertice
        self.distance_last_vertice, self.distance_next_vertice = self.distance_next_vertice, self.distance_last_vertice
    
    
    
    