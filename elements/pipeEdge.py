import numpy as np

class Edge():
    def __init__(self, vertice1, vertice2):
        self.v1 = vertice1
        self.v2 = vertice2
        self.robot = None
        self.color = 'black'
        self.v1.connect(self.v2)
        self.length = np.linalg.norm(self.v1.position - self.v2.position)
            
        self.robot_num = 0
    
    def view(self):
        if self.robot_num > 0:
            self.color = 'yellow'
        else:
            self.color = 'blue'
        # viewer
    