import numpy as np
import time
import gurobipy as gp
from gurobipy import GRB

start_state = np.array([[-4., 0., 0.], [0., -4., np.pi/2], [4., 0., np.pi], [0., 4., -np.pi/2]])
end_state = np.array([[4., 0.], [0., 4.], [-4., 0.], [0., -4.]])
all_radius = [0.2, 0.3, 0.2, 0.3]


class Opt():
    def __init__(self, i, dt):
        self.robot_state = start_state[i]
        self.orientation = start_state[i][2]
        self.target_position = end_state[i]
        self.robot_radius = all_radius[i]
        self.neighbor_robots = np.array([[0., -4., np.pi/2, 0., 0., 0.3], [4., 0., np.pi, 0., 0., 0.2], [0., 4., -np.pi/2, 0., 0., 0.3]])
        self.v_round = [0., 0.5]
        self.w_round = [-1., 1.]
        self.dt = dt
        self.m = None
        self.v = None
        self.w = None
        self.x_pred = None
        self.y_pred = None
        self.sin_cos_ref = None
        self.x_delta = None
        self.y_delta = None
        self.ep = []
        self.create_opt()
        self.v_output = None
        self.w_output = None
        
    def create_opt(self):
        self.m = gp.Model("local_solve")
        self.v = self.m.addVar(lb=self.v_round[0], ub=self.v_round[1], name="v")
        self.w = self.m.addVar(lb=self.w_round[0], ub=self.w_round[1], name="w")
        self.x_pred = self.m.addVar(lb=-GRB.INFINITY, ub=GRB.INFINITY, name="xp")
        self.y_pred = self.m.addVar(lb=-GRB.INFINITY, ub=GRB.INFINITY, name="yp")
        self.x_delta = self.m.addVar(lb=-GRB.INFINITY, ub=GRB.INFINITY, name="xd")
        self.y_delta = self.m.addVar(lb=-GRB.INFINITY, ub=GRB.INFINITY, name="yd")
        self.sin_cos_ref = self.m.addVar(lb=-GRB.INFINITY, ub=GRB.INFINITY, name="sin_cos_ref")

        for i in range(len(start_state)-1):
            self.ep = self.ep + [self.m.addVar(lb=0, ub=GRB.INFINITY, name="ep"+str(i))]
        self.ep=np.array(self.ep).reshape((1, len(start_state)-1))   

        for i in range(len(self.neighbor_robots)-1):
            self.m.addConstr((self.x_pred - self.neighbor_robots[i][0])**2 + (self.y_pred - self.neighbor_robots[i][1])**2 - (self.robot_radius + self.neighbor_robots[i][5])**2 - self.ep[0][i] >= 0)
        
        # self.m.addConstr(self.robot_state[0] + self.v * np.cos(self.orientation + self.w*self.dt/2) == self.x_pred)
        # self.m.addConstr(self.robot_state[1] + self.v * np.sin(self.orientation + self.w*self.dt/2) == self.y_pred)
        self.m.addConstr(2 * self.orientation + self.w * self.dt == 2 * self.sin_cos_ref)
        self.m.addConstr(self.x_pred - self.robot_state[0] == self.v * self.x_delta * self.dt)
        self.m.addConstr(self.y_pred - self.robot_state[1] == self.v * self.y_delta * self.dt)
        self.m.addGenConstrSin(self.sin_cos_ref, self.y_delta)
        self.m.addGenConstrCos(self.sin_cos_ref, self.x_delta)
        
        self.m.setObjective((self.x_pred - self.target_position[0])**2 + (self.y_pred - self.target_position[1])**2, GRB.MINIMIZE)
        
        self.m.setParam(GRB.Param.NonConvex, 2)
        
        # self.m.update()
        self.m.write("tmc_coll.lp")
        self.m.update()
        self.m.optimize()


        print("*********************************************************")
        print("Time to solve (ms)=",self.m.runtime*1000)
        print("*********************************************************")

        if self.m.status == GRB.Status.OPTIMAL:
            print('Optimal Solution found')
            
            print("linear_velocity:", self.v.X, " angular_velocity:", self.w.X)
            self.v_output = self.v.X
            self.w_output = self.w.X
            
            return True
        elif self.m.status == GRB.Status.INF_OR_UNBD:
            print('Model is infeasible or unbounded')
            return False
        elif self.m.status == GRB.Status.INFEASIBLE:
            print('Model is infeasible')
            return False
        elif self.m.status == GRB.Status.UNBOUNDED:
            print('Model is unbounded')
            return False
        else:
            print('Optimization ended with status %d' % self.m.status)
            return False

        
            
if __name__ == "__main__":
    solver = Opt(0, 0.2)