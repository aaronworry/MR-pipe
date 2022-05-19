
class Robot():
    def __init__(self, env, init_position, ori):
        self.orientation = ori
        self.position = init_position
        self.edge = None
        self.last_vertice = None
        self.distance_last_vertice = 0
        self.distance_next_vertice = 0        
        self.next_vertice = None
        self.velocity = 0
        self.color = None
        
    def cal_edge(self):
        # what the edge that this robot in.
        # self.position
        
        pass
        
    def move(self, velocity, position, orientation):
        pass
        
    def turn(self):
        # data in self.next_vertice.neighbor could be choosen to turn
        pass
    
    
    def view(self):
        self.color = 'green'
        pass