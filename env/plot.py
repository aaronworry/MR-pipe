import numpy as np
import matplotlib as mpl
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

class env_viewer():
    def __init__(self, env):
        
        self.width = env.width
        self.height = env.height
        self.offset_x = env.offset_x
        self.offset_y = env.offset_y
        self.layer_num = env.layer_num
        self.fig, self.axes = plt.subplots(nrows = 1, ncols = self.layer_num)
        
        self.color_list = ['g', 'b', 'r', 'c', 'm', 'y', 'k', 'w']
        
        self.robots = env.robots
        self.pipeVertices = env.vertices
        self.pipeEdges = env.Pipe
        
        self.nodes = env.nodes
        
    def init_viewer(self):
        for ax in self.axes:
            ax.set_aspect('equal')
            ax.set_xlim(self.offset_x, self.offset_x + self.width)
            ax.set_ylim(self.offset_y, self.offset_y + self.height)
            ax.set_xlabel("x [m]")
            ax.set_ylabel("y [m]")
        self.draw_pipe()
        self.show()
    
    def draw_pipe(self):
        self.drawPipeEdges(self.pipeEdges)
        self.drawPipeVertices(self.nodes)
        
    def update(self):
        pass
    
    def com_cla(self):
        # self.ax.patches = []
        self.ax.texts.clear()
        '''
        for pipeEdge_plot in self.pipeEdge_plot_list:
            pipeEdge_plot.remove()

        for pipeVertice_plot in self.pipeVertice_plot_list:
            pipeVertice_plot.remove()
        '''
        for robot_plot in self.robot_plot_list:
            robot_plot.remove()


    
    def cla(self, ax):
        ax.cla()

    def pause(self, time=0.001):
        plt.pause(time)
        
    def show(self):
        plt.show()
        
    def drawPipeEdges(self, edges):
        for aspect_edge in edges:
            self.drawPipeEdge_aspect(aspect_edge)
        
    def drawPipeVertices(self, vertices):
        # for aspect_vertice in vertices:
        self.drawPipeVertice_aspect(vertices) 
        
    def drawRobots(self, robots):
        for robot in robots:
            self.drawRobot(robot)
        
    def drawPipeEdge_aspect(self, edges):
        for item in edges:
            layer_item = item[0][2]
            self.line_plot(self.axes[layer_item], item, markersize=2, color="k")
            
        
    def drawPipeVertice_aspect(self, vertices):
        for item in vertices:
            layer_item = item.position[2]
            if item.neighbor[4] > 0 or item.neighbor[5] > 0:
                self.point_plot(self.axes[layer_item], item.position, markersize=2, color="green")
            else:
                self.point_plot(self.axes[layer_item], item.position, markersize=2, color="black")
            
        
    def drawRobot(self, robot):
        pass
        
    def point_plot(self, ax, point, markersize=2, color="black"):
        # pipe vertice
        x = point[0]
        y = point[1]           
    
        ax.plot([x], [y], marker='o', markersize=markersize, color=color)
    
    def point_arrow_plot(self, ax, point, length=0.5, width=0.3, color='red'):
        # robot
        px = point[0, 0]
        py = point[1, 0]
        theta = point[2, 0]

        pdx = length * cos(theta)
        pdy = length * sin(theta)

        point_arrow = mpl.patches.Arrow(x=px, y=py, dx=pdx, dy=pdy, color=color, width=width)

        ax.add_patch(point_arrow)
    
    def line_plot(self, ax, line, markersize=2, color="black"):
        # pipe  line  (2, 3)
        ax.plot([line[0][0], line[1][0]], [line[0][1], line[1][1]], marker='o', markersize=markersize, color=color)
        
        