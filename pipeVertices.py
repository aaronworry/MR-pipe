import numpy as np

class Vertice():
    def __init__(self, env, position):
        self.env = env
        self.position = position
        self.neighbor = [0, 0, 0, 0, 0, 0] #[+x, -x, +y, -y, +z, -z]
        # z-axis
        self.up = None  #+
        self.down = None
        # x-axis
        self.front = None #+
        self.rear = None
        # y-axis
        self.left = None 
        self.right = None #+
        
    def connect(self, vertice):
        dis = np.linalg.norm(self.position - vertice.position)
        
        if dis == self.position[0] - vertice.position[0]:
            if self.neighbor[1] == 0 and vertice.neighbor[0] == 0:
                self.rear = vertice
                self.neighbor[1] = 1
                vertice.front = self
                vertice.neighbor[0] = 1
            else:
                print('it has another rear vertice')
        elif dis == vertice.position[0] - self.position[0]:
            if self.neighbor[0] == 0 and vertice.neighbor[1] == 0:
                self.front = vertice
                self.neighbor[0] = 1
                vertice.rear = self
                vertice.neighbor[1] = 1
            else:
                print('it has another front vertice')
                
        elif dis == self.position[1] - vertice.position[1]:
            if self.neighbor[3] == 0 and vertice.neighbor[2] == 0:
                self.left = vertice
                self.neighbor[3] = 1
                vertice.right = self
                vertice.neighbor[2] = 1
            else:
                print('it has another left vertice')
        elif dis == vertice.position[1] - self.position[1]:
            if self.neighbor[2] == 0 and vertice.neighbor[3] == 0:
                self.right = vertice
                self.neighbor[2] = 1
                vertice.left = self
                vertice.neighbor[3] = 1
            else:
                print('it has another right vertice')
                
        elif dis == self.position[2] - vertice.position[2]:
            if self.neighbor[5] == 0 and vertice.neighbor[4] == 0:
                self.down = vertice
                self.neighbor[5] = 1
                vertice.up = self
                vertice.neighbor[4] = 1
            else:
                print('it has another down vertice')
        elif dis == vertice.position[2] - self.position[2]:
            if self.neighbor[4] == 0 and vertice.neighbor[5] == 0:
                self.up = vertice
                self.neighbor[4] = 1
                vertice.down = self
                vertice.neighbor[5] = 1
            else:
                print('it has another up vertice')
                
        else:
            print('it can not connect this vertice')
    
    def view(self):
        pass