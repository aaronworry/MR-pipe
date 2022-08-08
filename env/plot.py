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
        self.robot_plot_list = []
        
        self.robots = env.robots
        self.pipeVertices = env.vertices   #nodes
        self.pipeEdges = env.horizonPipe # edges
        self.edges = env.edges
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
        self.drawPipeEdges(self.edges)
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
        self.robot_plot_list = []


    
    def cla(self, ax):
        ax.cla()

    def pause(self, time=0.001):
        plt.pause(time)
        
    def show(self):
        plt.show()
        
    def drawPipeEdges(self, edges):
        for aspect_edge in edges:
            self.drawPipeEdge(aspect_edge)
        
    def drawPipeVertices(self, vertices):
        for vertice in vertices:
            self.drawPipeVertice(vertice)
        
    def drawRobots(self, robots):
        for robot in robots:
            self.drawRobot(robot)
            
    def drawPipeEdge(self, edge):
        layer_item1 = edge.v1.position[2]
        layer_item2 = edge.v2.position[2]
        if layer_item1 == layer_item2:
            self.line_plot2(self.axes[layer_item1], edge, markersize=2, color="k")
            # self.axes[layer_item1].plot([edge.v1.position[0], edge.v2.position[0]], [edge.v1.position[1], edge.v2.position[1]], marker='o', markersize=2, color='black')
            
        
    def drawPipeVertice(self, item):
        layer_item = item.position[2]
        if item.neighbor[4] > 0 or item.neighbor[5] > 0:
            self.point_plot(self.axes[layer_item], item.position, markersize=4, color="red")
        else:
            self.point_plot(self.axes[layer_item], item.position, markersize=2, color="black")
            
        
    def drawRobot(self, robot):
        robot_circle = mpl.patches.Circle(xy=(x, y), radius = robot.radius, color = robot_color)
        robot_circle.set_zorder(2)
        #decide which ax it would be drawn. if the robot is in vertical pipe, it would be drawn in the ax where the vertice in front of the robot.
        self.ax.add_patch(robot_circle)
        self.robot_plot_list.append(robot_circle)
        # 6 ori
        arrow = None
        # 朝上:点
        if robot.orientation == :
            arrow = mpl.patches.Arrow(x, y, 0.5*cos(theta), 0.5*sin(theta), width = 0.6)
            
        # 朝下:×
        elif 
        # 其他朝向:箭头
        elif
        
        arrow.set_zorder(1)
        self.ax.add_patch(arrow)
        self.robot_plot_list.append(arrow)
        
        
    def point_plot(self, ax, point, markersize=2, color="black"):
        # pipe vertice
        x = point[0]
        y = point[1]           
    
        ax.plot([x], [y], marker='o', markersize=markersize, color=color)
    
    def point_arrow_plot(self, ax, point, length=0.5, width=0.3, color='red'):
        # robot
        px = point[0]
        py = point[1]
        theta = point[2]

        pdx = length * cos(theta)
        pdy = length * sin(theta)

        point_arrow = mpl.patches.Arrow(x=px, y=py, dx=pdx, dy=pdy, color=color, width=width)

        ax.add_patch(point_arrow)
    
    def line_plot(self, ax, line, markersize=2, color="black"):
        # pipe  line  (2, 3)
        ax.plot([line[0][0], line[1][0]], [line[0][1], line[1][1]], marker='o', markersize=markersize, color=color)
        
    def line_plot2(self, ax, edge, markersize=2, color="black"):
        # pipe  edge
        ax.plot([edge.v1.position[0], edge.v2.position[0]], [edge.v1.position[1], edge.v2.position[1]], marker='o', markersize=markersize, color=color)
        
        