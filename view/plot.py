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
        # self.ax.patches = []
        # self.ax.texts.clear()
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
        # plt.show()
        # self.fig.canvas.draw()
        plt.draw()
        
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
            # self.line_plot2(self.axes[layer_item1], edge, markersize=2, color=edge.color)
            self.line_plot(self.axes[layer_item1], edge, markersize=2, color=edge.color)
            # self.axes[layer_item1].plot([edge.v1.position[0], edge.v2.position[0]], [edge.v1.position[1], edge.v2.position[1]], marker='o', markersize=2, color='black')
            
        
    def drawPipeVertice(self, item):
        layer_item = item.position[2]
        if item.neighbor[4] > 0 or item.neighbor[5] > 0:
            self.point_plot(self.axes[layer_item], item.position, radius=0.1, color="red")
        else:
            self.point_plot(self.axes[layer_item], item.position, radius=0.08, color="black")
            
        
    def drawRobot(self, robot):
        robot_circle = mpl.patches.Circle(xy=(robot.position[0], robot.position[1]), radius = 0.15, color = robot.color)
        robot_circle.set_zorder(2)
        #decide which ax it would be drawn. if the robot is in vertical pipe, it would be drawn in the ax where the vertice in front of the robot.
        temp = robot.position[2]
        layer_item = 0
        if temp == 0 or temp == 1:
            layer_item = temp
        else:
            if robot.orientation[4] == 1:
                layer_item = 1
            elif robot.orientation[5] == 1:
                layer_item = 0
        
        self.axes[layer_item].add_patch(robot_circle)
        self.robot_plot_list.append(robot_circle)
        # 6 ori
        arrow = mpl.patches.Circle(xy=(robot.position[0], robot.position[1]), radius = 0.1, color = robot.color)
        # up: yellow
        if robot.orientation[4] == 1:
            arrow = mpl.patches.Circle(xy=(robot.position[0], robot.position[1]), radius = 0.1, color = 'yellow')
        # down: purple
        elif robot.orientation[5] == 1:
            arrow = mpl.patches.Circle(xy=(robot.position[0], robot.position[1]), radius = 0.1, color = 'darkviolet')
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
    
    def point_arrow_plot(self, ax, point, length=0.5, width=0.3, color='red'):
        # robot
        px = point[0]
        py = point[1]
        theta = point[2]

        pdx = length * cos(theta)
        pdy = length * sin(theta)

        point_arrow = mpl.patches.Arrow(x=px, y=py, dx=pdx, dy=pdy, color=color, width=width)

        ax.add_patch(point_arrow)
    
    def line_plot(self, ax, edge, markersize=2, color="black"):
        # pipe  line  (2, 3)
        line = mpl.lines.Line2D([edge.v1.position[0], edge.v2.position[0]], [edge.v1.position[1], edge.v2.position[1]], marker='o', markersize=markersize, color=color)
        ax.add_line(line)
        self.pipeEdge_plot_list.append(line)
        
    def line_plot2(self, ax, edge, markersize=2, color="black"):
        # pipe  edge
        ax.plot([edge.v1.position[0], edge.v2.position[0]], [edge.v1.position[1], edge.v2.position[1]], marker='o', markersize=markersize, color=color)
        
    # animation method 1
    def animate(self):

        self.draw_robot_diff_list()

        return self.ax.patches + self.ax.texts + self.ax.artists

    def show_ani(self):
        ani = animation.FuncAnimation(
        self.fig, self.animate, init_func=self.init_plot, interval=100, blit=True, frames=100, save_count=100)
        plt.show()
    
    def save_ani(self, name='animation'): 
        ani = animation.FuncAnimation(
        self.fig, self.animate, init_func=self.init_plot, interval=1, blit=False, save_count=300)
        ani.save(name+'.gif', writer='pillow')

    # # animation method 2
    def save_gif_figure(self, path, i, format='png'):

        if path.exists():
            order = str(i).zfill(3)
            plt.savefig(str(path)+'/'+order+'.'+format, format=format)
        else:
            path.mkdir()
            order = str(i).zfill(3)
            plt.savefig(str(path)+'/'+order+'.'+format, format=format)

    def create_animate(self, image_path, ani_path, ani_name='animated', keep_len=30, rm_fig_path=True):

        if not ani_path.exists():
            ani_path.mkdir()

        images = list(image_path.glob('*.png'))
        images.sort()
        image_list = []
        for i, file_name in enumerate(images):

            if i == 0:
                continue

            image_list.append(imageio.imread(file_name))
            if i == len(images) - 1:
                for j in range(keep_len):
                    image_list.append(imageio.imread(file_name))

        imageio.mimsave(str(ani_path)+'/'+ ani_name+'.gif', image_list)
        print('Create animation successfully')

        if rm_fig_path:
            shutil.rmtree(image_path)    