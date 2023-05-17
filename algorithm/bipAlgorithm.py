import numpy as np
import time
import gurobipy as gp
from gurobipy import GRB



class BIPAlgorithm():
    def __init__(self):
        pass
        
    def create_opt(self):
        self.m = gp.Model("local_solve")
        
        '''
        add obj and cons
        '''
        
        self.m.setParam(GRB.Param.NonConvex, 2)
        self.m.update()
        self.m.write("test.lp")
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
    solver = Opt()