import numpy as np

import sys
sys.path.append("..")
import matplotlib.pyplot as plt

from elements.robot import Robot
from elements.pipeVertices import Vertice
from elements.pipeEdge import Edge
from plot import env_viewer

vertices0 = np.array([[0, 0, 0], [0, -1, 0], [0, -2, 0], [0, -3, 0], [1, 0, 0], [1, -3, 0], [3, 0, 0], [3, -3, 0]])
vertices1 = np.array([[0, 0, 1], [0, -1, 1], [0, -2, 1], [0, -3, 1], [1, 0, 1], [1, -1, 1], [1, -2, 1], [1, -3, 1]])
    
Edge0 = np.asarray([np.asarray([vertices0[0], vertices0[1]]), np.asarray([vertices0[2], vertices0[3]]), np.asarray([vertices0[4], vertices0[6]]), np.asarray([vertices0[5], vertices0[7]])])
Edge1 = np.asarray([np.asarray([vertices1[0], vertices1[1]]), np.asarray([vertices1[1], vertices1[2]]), np.asarray([vertices1[2], vertices1[3]]), np.asarray([vertices1[4], vertices1[5]]), np.asarray([vertices1[5], vertices1[6]]), np.asarray([vertices1[6], vertices1[7]]), np.asarray([vertices1[1], vertices1[5]]), np.asarray([vertices1[2], vertices1[6]])])
Edge0_1 = np.asarray([np.asarray([vertices0[0], vertices1[0]]), np.asarray([vertices0[3], vertices1[3]]), np.asarray([vertices0[4], vertices1[4]]), np.asarray([vertices0[5], vertices1[7]])])

pipeDict2 = {'vertices': [vertices0, vertices1], 'verticalEdge': [Edge0_1], 'horizonEdge': [Edge0, Edge1], 'Edge': [Edge0, Edge1, Edge0_1]}

ROBOTS = [np.array([0, -1, 0]), [0, 0, 1, 0, 0, 0], np.array([0, -2, 0]), [0, 0, 0, 1, 0, 0]]

class env():
    def __init__(self, dt, pipeDict, robot_num=2):
        self.robot_num = robot_num
        self.robots = []
        self.vertices = pipeDict['vertices']
        self.verticalPipe = pipeDict['verticalEdge']
        self.horizonPipe = pipeDict['horizonEdge']
        self.Pipe = pipeDict['Edge']
        
        self.width = 10
        self.height = 10
        self.offset_x = -2
        self.offset_y = -5
        self.layer_num = 2
        
        self.nodes = []
        self.edges = []
        
        self.plot = env_viewer(self)
        self._create_pipe_scenario()
        self._create_robots(2)
     
    def _create_pipe_scenario(self):
        # 初始化节点对象
        for vertice_ar in self.vertices:
            for item in vertice_ar:
                node = Vertice(self, item)
                self.nodes.append(node)
        # 初始化管道
        for edge_ar in self.Pipe:
            for item in edge_ar:   #(2, 3)
                temp1 = item[0]
                temp2 = item[1]
                flag1, flag2 = None, None
                for node in self.nodes:
                    if np.array_equal(node.position, temp1):
                        flag1 = node
                    elif np.array_equal(node.position, temp2):
                        flag2 = node
                    if flag1 is not None and flag2 is not None:
                        edge = Edge(flag1, flag2)
                        self.edges.append(edge)
                        break
        # 可视化界面加载
        self.plot.init_viewer()
        
    def _create_robots(self, robot_num):
        # 初始化机器人
        for i in range(robot_num):
            robot = Robot(self, ROBOTS[2*i], ROBOTS[2*i+1])
            self.robots.append(robot)  
        # 可视化机器人
    
        
    def reset(self):
        for robot in self.robots:
            robot.reset()
        
    def step(self):
        for robot in self.robots:
            robot.step()
        self.render()
        
    def viewer(self):
        pass
        
    def render(self, dt):
        self.plot.com_cla()
        self.plot.drawRobots(self.robots)
        self.plot.pause(dt)
        self.time += dt
        
    def show(self):
        self.plot.drawRobots(self.robots)
        self.plot.show()
        
        
if __name__ == '__main__':
    env = env(dt = 2, pipeDict = pipeDict2)
    
    