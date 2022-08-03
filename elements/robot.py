# 由 env 判断机器人在哪个管道内
class Robot():
    def __init__(self, env, init_position, ori):
        self.env = env
        self.init_orientation = ori # [0, 0, 0, 0, 0, 0]
        self.init_position = init_position
        self.velocity = 0
        self.orientation = None
        self.position = None
        
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

        self.color = None
        
        self.reset()
        
    def reset(self):
        pass
        
    def move(self, velocity):
        self.afterDis = velocity * self.env.dt - self.distance_next_vertice
        self.backDis = self.distance_last_vertice + velocity * self.env.dt
        if self.afterDis > 0:
            self.turnFlag = True
            self.turn()
        elif self.backDis < 0:
            self.turnBack() # 后退转向
        else:
            self.position = None
            self.distance_last_vertice += velocity * self.env.dt
            self.distance_next_vertice -= velocity * self.env.dt        
            
            
    def turn(self):
        # data in self.next_vertice.neighbor could be choosen to turn
        self.orientation = self.nextOri
        self.position = None
        
        self.last_vertice = self.next_vertice
        self.distance_last_vertice = self.afterDis 
        self.distance_next_vertice = None               
        self.next_vertice = None
    
    def turnBack(self):
        pass
        
    def check_collision(self):
        pass
    
    def action(self):
        
        return velocity, nextOri
    
    
    def view(self):
        self.color = 'green'
        pass