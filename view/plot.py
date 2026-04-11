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
        self.pipeEdge_plot_list = []
        
        self.pipe_path = env.pipe_path
        
        self.robots = env.robots
        self.edges = env.edges
        self.nodes = env.nodes
        
    def init_viewer(self):
        """
        if self.layer_num == 1:
            self.axes.set_aspect('equal')
            self.axes.set_xlim(self.offset_x, self.offset_x + self.width)
            self.axes.set_ylim(self.offset_y, self.offset_y + self.height)
            self.axes.set_xlabel("x [m]")
            self.axes.set_ylabel("y [m]")
        else:
            for ax in self.axes:
                ax.set_aspect('equal')
                ax.set_xlim(self.offset_x, self.offset_x + self.width)
                ax.set_ylim(self.offset_y, self.offset_y + self.height)
                ax.set_xlabel("x [m]")
                ax.set_ylabel("y [m]")
        """
        self.draw_pipe()
        
    
    def draw_pipe(self):
        self.drawPipeEdges(self.edges)
        self.drawPipeVertices(self.nodes)
        
    def upgrade_pipe_path(self):
        for pipeEdge_plot in self.pipeEdge_plot_list:
            pipeEdge_plot.remove()
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
            if self.layer_num == 1:
                self.axes.set_xlabel("x(m)", fontproperties = myfont)
                self.axes.set_ylabel("y(m)", fontproperties = myfont)
                self.axes.set_xticks([-1, 0, 1, 2, 3, 4], fontproperties = myfont)
                self.axes.set_yticks([-2, -1, 0, 1, 2, 3], fontproperties = myfont)
                self.axes.set_xticklabels([-1, 0, 1, 2, 3, 4], fontproperties = myfont)
                self.axes.set_yticklabels([-2, -1, 0, 1, 2, 3], fontproperties = myfont)
                self.axes.spines['right'].set_visible(False)
                self.axes.spines['top'].set_visible(False)
            else:
                for ax in self.axes:
                    ax.set_xlabel("x(m)", fontproperties = myfont)
                    ax.set_ylabel("y(m)", fontproperties = myfont)
                    ax.set_xticks([-1, 0, 1, 2, 3, 4], fontproperties = myfont)
                    ax.set_yticks([-2, -1, 0, 1, 2, 3], fontproperties = myfont)
                    ax.set_xticklabels([-1, 0, 1, 2, 3, 4], fontproperties = myfont)
                    ax.set_yticklabels([-2, -1, 0, 1, 2, 3], fontproperties = myfont)
                    ax.spines['right'].set_visible(False)
                    ax.spines['top'].set_visible(False)
                    # self.ax.grid(False)
                    # plt.xticks(fontproperties = myfont)
                    # plt.yticks(fontproperties = myfont)
                    # plt.axis('off')
            plt.savefig('../figures/result' + self.pipe_path[-6] + '.pdf',dpi=300,bbox_inches = "tight")
        
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
        layer_item1 = int(edge.v1.position[2])
        layer_item2 = int(edge.v2.position[2])
        if layer_item1 == layer_item2:

            if self.layer_num == 1:
                self.line_plot(self.axes, edge, markersize=2, color=edge.color)
            else:
                self.line_plot(self.axes[layer_item1], edge, markersize=2, color=edge.color)
        
        
    def drawPipeVertice(self, item):
        layer_item = int(item.position[2])
        if item.neighbor_flag[4] > 0 or item.neighbor_flag[5] > 0:
            if self.layer_num == 1:
                self.point_plot(self.axes, item.position, radius=0.1, color="red")
            else:
                self.point_plot(self.axes[layer_item], item.position, radius=0.1, color="red")
        else:
            if self.layer_num == 1:
                self.point_plot(self.axes, item.position, radius=0.08, color="black")
            else:
                self.point_plot(self.axes[layer_item], item.position, radius=0.08, color="black")
            
        
    def drawRobot(self, robot):
        robot_circle = mpl.patches.Circle(xy=(robot.position[0], robot.position[1]), radius = 0.15, color = robot.color)
        robot_circle.set_zorder(2)
        #decide which ax it would be drawn. if the robot is in vertical pipe, it would be drawn in the ax where the vertice in front of the robot.
        temp = int(robot.position[2])
        layer_item = 0
        if temp == 0 or temp == 1:
            layer_item = temp
        else:
            if robot.orientation[4] == 1:
                layer_item = 1
            elif robot.orientation[5] == 1:
                layer_item = 0
        
        if self.layer_num == 1:
            self.axes.add_patch(robot_circle)
        else:
            self.axes[layer_item].add_patch(robot_circle)
        self.robot_plot_list.append(robot_circle)
        # 6 ori
        arrow = mpl.patches.Circle(xy=(robot.position[0], robot.position[1]), radius = 0.1, color = robot.color)
        # up: yellow
        if robot.orientation[4] == 1:
            arrow = mpl.patches.Circle(xy=(robot.position[0], robot.position[1]), radius = 0.1, color = 'indigo')
        # down: purple
        elif robot.orientation[5] == 1:
            arrow = mpl.patches.Circle(xy=(robot.position[0], robot.position[1]), radius = 0.1, color = 'deeppink')
        # other ori: arrow
        elif robot.orientation[0] == 1:
            arrow = mpl.patches.Arrow(robot.position[0], robot.position[1], 0.3, 0, width = 0.1, color = robot.color)
        elif robot.orientation[1] == 1:
            arrow = mpl.patches.Arrow(robot.position[0], robot.position[1], -0.3, 0, width = 0.1, color = robot.color)
        elif robot.orientation[2] == 1:
            arrow = mpl.patches.Arrow(robot.position[0], robot.position[1], 0, 0.3, width = 0.1, color = robot.color)
        elif robot.orientation[3] == 1:
            arrow = mpl.patches.Arrow(robot.position[0], robot.position[1], 0, -0.3, width = 0.1, color = robot.color)
        
        arrow.set_zorder(3)
        if self.layer_num == 1:
            self.axes.add_patch(arrow)
        else:
            self.axes[layer_item].add_patch(arrow)
        self.robot_plot_list.append(arrow)
        
        
    def point_plot(self, ax, point, radius, color="black"):
        # pipe vertice
        x = point[0]
        y = point[1]     

        vertice_circle = mpl.patches.Circle(xy=(x, y), radius = radius, color = color)
        vertice_circle.set_zorder(1)
        ax.add_patch(vertice_circle)
    
        # ax.plot([x], [y], marker='o', markersize=markersize, color=color)
    
    
    def line_plot(self, ax, edge, markersize=2, color="black"):
        # pipe  line  (2, 3)
        line = mpl.lines.Line2D([edge.v1.position[0], edge.v2.position[0]], [edge.v1.position[1], edge.v2.position[1]], marker='o', markersize=markersize, color=color)
        ax.add_line(line)
        self.pipeEdge_plot_list.append(line)
          
