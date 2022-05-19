import numpy as np

class Edge():
    def __init__(self, vertice1, vertice2):
        self.v1 = vertice1
        self.v2 = vertice2
        self.robot = None
        self.color = 'red'
        self.v1.connect(self.v2)
        self.length = np.linalg.norm(self.v1.position, self.v2.position)
            
    def set_robot(self, robot):
        self.robot = robot
    
    def view(self):
        if self.robot:
            self.color = 'blue'
        else:
            self.color = 'red'
        # viewer
    