import numpy as np
import time

import sys
sys.path.append("..")
import matplotlib.pyplot as plt

from elements.robot import Robot
from elements.pipeVertices import Vertice
from elements.pipeEdge import Edge
from view.plot import env_viewer
from view.plot3D import env_viewer_3D
from algorithm.weightGraph import WeightGraph
from maps.getMap import GetMap

class Env():
    def __init__(self, dt, pipe_path, dim=2):
        self.robots = []
        self.dt = dt
        self.reward = 0
        self.pipe_path = pipe_path
        self.maps = GetMap(self.pipe_path)
        self.vertices = self.maps.map_dict['vertices']
        self.Pipe = self.maps.map_dict['edges']
        self.ROBOTS = self.maps.map_dict['robots']
        self.ROBOTS_ORI = self.maps.map_dict['robots_ori']
        self.layer_num = self.maps.map_dict['layer_num']
        self.robot_num = len(self.ROBOTS)
        
        
        self.width = self.maps.map_dict['width'][0]
        self.height = self.maps.map_dict['height'][0]
        self.offset_x = self.maps.map_dict['offset_x'][0]
        self.offset_y = self.maps.map_dict['offset_y'][0]
        
        self.time = 0
        self.done = False
        self.robot_finihsed_set = set()
        self.color = ['green', 'blue', 'cyan', 'purple', 'yellow']
        
        self.nodes = []
        self.edges = []
        
        self.edge_passed = set()
        self.edge_not_passed = set()
        self.all_edges = set()
        self.collision_finished_robots = set()
        
        self.plot = None
        if dim == 3:
            self.plot = env_viewer_3D(self)
        else:
            self.plot = env_viewer(self)
        self._create_pipe_scenario()
        self._create_robots()
        
        self.graph = WeightGraph(self.vertices, self.Pipe)
        self.plot.show(True)
    
    def _create_pipe_scenario(self):
        # 初始化节点对象
        for vertice_ar in self.vertices:
            node = Vertice(self, np.asarray(vertice_ar))
            self.nodes.append(node)
        # 初始化管道
        for edge_ar in self.Pipe:
            temp1 = np.asarray(edge_ar[0])
            temp2 = np.asarray(edge_ar[1])
            flag1, flag2 = None, None
            for node in self.nodes:
                if np.array_equal(node.position, temp1):
                    flag1 = node
                elif np.array_equal(node.position, temp2):
                    flag2 = node
                if flag1 is not None and flag2 is not None:
                    edge = Edge(flag1, flag2)
                    self.edges.append(edge)
                    self.all_edges.add(edge)
                    break
        # 可视化界面加载
        self.plot.init_viewer()
    
    """
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
                        self.all_edges.add(edge)
                        break
        # 可视化界面加载
        self.plot.init_viewer()
    """
        
    def _create_robots(self):
        # 初始化机器人
        for i in range(self.robot_num):
            robot = Robot(self, i, np.asarray(self.ROBOTS[i]), self.ROBOTS_ORI[i])
            robot.color = self.color[(i+1)%len(self.color)]
            self.robots.append(robot)  
        # 可视化机器人
        self.plot.drawRobots(self.robots)
        # self.plot.show()

    def path_planning(self, paths):
        # task_allocation   为机器人分配路径, 并更新颜色
        for path in paths:
            true_path = path['path']
            start_position = self.graph.get_position_of_node(true_path[0])
            for robot in self.robots:
                if np.array_equal(robot.position, start_position) and robot.allocated == False:
                    # self.upgrade_edges(path, robot.color)
                    robot.assign_path(true_path)
                    robot.allocated = True
                
        # self.plot.upgrade_pipe_path()
        self.render()
    
    def upgrade_edges(self, edges, color):
        # 为规划的路径标注颜色
        for item in edges:
            for k in self.edges:
                if np.array_equal(k.v1.position, item[0]) and np.array_equal(k.v2.position, item[1]):
                    k.color = color
                elif np.array_equal(k.v1.position, item[1]) and np.array_equal(k.v2.position, item[0]):
                    k.color = color
    
    
    def reset(self):
        for robot in self.robots:
            robot.reset()
        self.robot_finihsed_set = set()
        self.edge_passed = set()
        self.collision_finished_robots = set()
        self.edge_not_passed = self.all_edges
        
    def step_path(self):
        for robot in self.robots:
            # env 负责 prior 的减少，robot的path不包括 path_id时 且 prior 为空时， robot 告知env ，让env删掉其它机器人的prior的自己
            robot.prior_robots -= self.collision_finished_robots
        self.collision_finished_robots = set()
        for robot in self.robots:
            robot.collision_avoidance()
        for robot in self.robots:
            if robot.conflict_resolution_flag == True:
                robot.step_avoidance_path()
            elif robot.conflict_resolution_forward_flag == True:
                robot.step_avoidance_forward_path()
            else:
                robot.step_path()
        self.render()
    
    def step(self, actions):
        for i in range(len(self.robots)):
            self.robots[i].step(actions[i])
        
        self.communication()  # update robot.edge_passed 
        
        for robot in self.robots:
            self.edge_passed = self.edge_passed | robot.edge_passed
            self.edge_not_passed = self.edge_not_passed & robot.edge_not_passed
        # self.edge_not_passed = self.all_edges - self.edge_passed
        self.render()
        
    def observation(self):
        pass
        
    def get_reward_and_termination(self):
        pass
    
    def communication(self):
        for robot in self.robot:
            robot.communicate(robot.neighbor_robots)
    
    def render(self):
        self.plot.com_cla()  
        self.plot.drawRobots(self.robots)
        self.plot.show()
        self.plot.pause(self.dt)
        self.time += self.dt
    
    """
    def save_fig(self, path, i):
        self.plot.save_gif_figure(path, i)
    
    def save_ani(self, image_path, ani_path, ani_name='animated', **kwargs):
        self.plot.create_animate(image_path, ani_path, ani_name=ani_name, **kwargs)

    def show(self, **kwargs):
        self.plot.draw_dyna_components(**kwargs)
        self.plot.show()
    
    def show_ani(self):
        self.plot.show_ani()
    """
        

    