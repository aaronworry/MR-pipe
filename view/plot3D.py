import numpy as np
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3D
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import imageio
import platform
import shutil
from matplotlib import image

from math import cos, sin, pi
from pathlib import Path
import inspect
import matplotlib.transforms as mtransforms

class env_viewer_3D():
    def __init__(self, env):
        
        self.width = env.width
        self.height = env.height
        self.offset_x = env.offset_x
        self.offset_y = env.offset_y

        self.fig = plt.figure()
        self.ax = self.fig.gca(projection='3d')
        
        self.color_list = ['g', 'b', 'r', 'c', 'm', 'y', 'k', 'w']
        self.robot_plot_list = []
        self.pipeEdge_plot_list = []
        
        self.robots = env.robots
        self.edges = env.edges
        self.nodes = env.nodes
        
    def init_viewer(self):
        self.ax.set_xlim(self.offset_x, self.offset_x + self.width)
        self.ax.set_ylim(self.offset_y, self.offset_y + self.height)
        self.ax.set_zlim(0., 2.)
        self.ax.set_xlabel("x [m]")
        self.ax.set_ylabel("y [m]")
        self.ax.set_zlabel("z [m]")
        # self.ax.spines['right'].set_visible(False)
        # self.ax.spines['top'].set_visible(False)
        self.draw_pipe()
        
    
    def draw_pipe(self):
        self.drawPipeEdges(self.edges)
        self.drawPipeVertices(self.nodes)
        
    def upgrade_pipe_path(self):
        for line in self.pipeEdge_plot_list:
            for item in line:
                item.remove()
                del item
        self.pipeEdge_plot_list = []
        self.drawPipeEdges(self.edges)
        
    def update(self):
        pass
    
    def com_cla(self):
        for robot_plot in self.robot_plot_list:
            robot_plot.remove()
        self.robot_plot_list = []


    
    def cla(self, ax):
        ax.cla()

    def pause(self, time=0.001):
        plt.pause(time)
        
    def show(self, flag=False):
        plt.draw()
        if flag:
            myfont = mpl.font_manager.FontProperties(fname=r"c:\windows\fonts\times.ttf", size=14)
            self.ax.set_xlabel("x(m)", fontproperties = myfont)
            self.ax.set_ylabel("y(m)", fontproperties = myfont)
            self.ax.set_zlabel("z(m)", fontproperties = myfont)
            self.ax.set_xticks([0, 1, 2, 3, 4, 5], fontproperties = myfont)
            self.ax.set_yticks([-1, 0, 1, 2, 3, 4, 5], fontproperties = myfont)
            self.ax.set_zticks([0, 1, 2], fontproperties = myfont)
            self.ax.set_xticklabels([0, 1, 2, 3, 4, 5], fontproperties = myfont)
            self.ax.set_yticklabels([-1, 0, 1, 2, 3, 4, 5], fontproperties = myfont)
            self.ax.set_zticklabels([0, 1, 2], fontproperties = myfont)
            self.ax.grid(False)
            # plt.xticks(fontproperties = myfont)
            # plt.yticks(fontproperties = myfont)
            # plt.axis('off')
            plt.savefig('../figures/result.pdf',dpi=300,bbox_inches = "tight")
            
        
    def drawPipeEdges(self, edges):
        for edge in edges:
            self.drawPipeEdge(edge)
        
    def drawPipeVertices(self, vertices):
        for vertice in vertices:
            self.drawPipeVertice(vertice)
        
    def drawRobots(self, robots):
        for robot in robots:
            self.drawRobot(robot)
            
    def drawPipeEdge(self, edge):
        self.line_plot(edge)    
        
    def drawPipeVertice(self, item):
        vertice_point = self.ax.scatter(item.position[0], item.position[1], item.position[2], marker = 'o', s = 4, color='black')
        
            
        
    def drawRobot(self, robot):
        self.point_plot(robot)
        self.point_arrow_plot(robot)

    def point_plot(self, robot):
        robot_point = self.ax.scatter(robot.position[0], robot.position[1], robot.position[2], marker = 'o', s = 20, color=robot.color)
        self.robot_plot_list.append(robot_point)
    
    def point_arrow_plot(self, robot):
        point_arrow = None
        if robot.orientation[0] == 1:
            point_arrow = self.ax.quiver(robot.position[0], robot.position[1], robot.position[2], 0.3, 0, 0, arrow_length_ratio=0.1, color=robot.color)
        elif robot.orientation[1] == 1:
            point_arrow = self.ax.quiver(robot.position[0], robot.position[1], robot.position[2], -0.3, 0, 0, arrow_length_ratio=0.1, color=robot.color)
        elif robot.orientation[2] == 1:
            point_arrow = self.ax.quiver(robot.position[0], robot.position[1], robot.position[2], 0, 0.3, 0, arrow_length_ratio=0.1, color=robot.color)
        elif robot.orientation[3] == 1:
            point_arrow = self.ax.quiver(robot.position[0], robot.position[1], robot.position[2], 0, -0.3, 0, arrow_length_ratio=0.1, color=robot.color)
        elif robot.orientation[4] == 1:
            point_arrow = self.ax.quiver(robot.position[0], robot.position[1], robot.position[2], 0, 0, 0.3, arrow_length_ratio=0.1, color=robot.color)
        elif robot.orientation[5] == 1:
            point_arrow = self.ax.quiver(robot.position[0], robot.position[1], robot.position[2], 0, 0, -0.3, arrow_length_ratio=0.1, color=robot.color)
        self.robot_plot_list.append(point_arrow)
    
    def line_plot(self, edge):
        line = self.ax.plot([edge.v1.position[0], edge.v2.position[0]], [edge.v1.position[1], edge.v2.position[1]], [edge.v1.position[2], edge.v2.position[2]], color = edge.color)
        self.pipeEdge_plot_list.append(line)