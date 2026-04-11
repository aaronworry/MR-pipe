import numpy as np

class Vertice():
    def __init__(self, env, position):
        self.env = env
        self.position = position
        self.neighbor_flag = [0, 0, 0, 0, 0, 0] #[+x, -x, +y, -y, +z, -z]
        self.neighbors = set()
        # z-axis
        self.up = None  #+
        self.down = None
        # x-axis
        self.front = None #+
        self.rear = None
        # y-axis
        self.left = None 
        self.right = None #+
        
        self.aspect = self.position[2]
        
    def connect(self, vertice):
        dis = np.linalg.norm(self.position - vertice.position)
        self.neighbors.add(vertice)
        vertice.neighbors.add(self)
        
        if dis == self.position[0] - vertice.position[0]:
            if self.neighbor_flag[1] == 0 and vertice.neighbor_flag[0] == 0:
                self.rear = vertice
                self.neighbor_flag[1] = 1
                vertice.front = self
                vertice.neighbor_flag[0] = 1
                # self.neighbors.append(vertice)
                # vertice.neighbors.append(self)
            else:
                print('it has another rear vertice')
        elif dis == vertice.position[0] - self.position[0]:
            if self.neighbor_flag[0] == 0 and vertice.neighbor_flag[1] == 0:
                self.front = vertice
                self.neighbor_flag[0] = 1
                vertice.rear = self
                vertice.neighbor_flag[1] = 1
            else:
                print('it has another front vertice')
                
        elif dis == self.position[1] - vertice.position[1]:
            if self.neighbor_flag[3] == 0 and vertice.neighbor_flag[2] == 0:
                self.left = vertice
                self.neighbor_flag[3] = 1
                vertice.right = self
                vertice.neighbor_flag[2] = 1
            else:
                print('it has another left vertice')
        elif dis == vertice.position[1] - self.position[1]:
            if self.neighbor_flag[2] == 0 and vertice.neighbor_flag[3] == 0:
                self.right = vertice
                self.neighbor_flag[2] = 1
                vertice.left = self
                vertice.neighbor_flag[3] = 1
            else:
                print('it has another right vertice')
                
        elif dis == self.position[2] - vertice.position[2]:
            if self.neighbor_flag[5] == 0 and vertice.neighbor_flag[4] == 0:
                self.down = vertice
                self.neighbor_flag[5] = 1
                vertice.up = self
                vertice.neighbor_flag[4] = 1
            else:
                print('it has another down vertice')
        elif dis == vertice.position[2] - self.position[2]:
            if self.neighbor_flag[4] == 0 and vertice.neighbor_flag[5] == 0:
                self.up = vertice
                self.neighbor_flag[4] = 1
                vertice.down = self
                vertice.neighbor_flag[5] = 1
            else:
                print('it has another up vertice')
                
        else:
            print('it can not connect this vertice')
    
    def view(self):
        pass