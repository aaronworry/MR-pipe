import time
import sys
import numpy as np

# cost so many time
class ExhaustiveSpaceToTime():
    def __init__(self, graph, ROBOT):        
        self.Graph = graph
        self.node = self.Graph.node
        self.get_start_ids(ROBOT)
        self.k = len(ROBOT)
        
        self.graph = [list(elem) for elem in self.Graph.get_edges()]
        self.A = self.get_adjance_matrix()
        self.single_robot_X_t = []
        self.case_X_t = []
        self.case_num = 0
        self.solutions = []
        
    def solve(self):
        self.iter_X_t()
        self.case_num = pow(len(self.single_robot_X_t), 2)
        self.cal_combination_case_k_robot(self.k)
        print(len(self.case_X_t), self.case_num)
        # cal X0 based on initial state
        X0 = None
        self.iter_solution((X0, ), 1)
        Result = self.solutions[0]
        return self.trans(Result)
    
    
    def trans(self, X):
        # X to path
        return None
        
    
    
    def get_adjance_matrix(self):
        result = np.zeros((self.node, self.node))
        for item in self.graph:
            result[item[0]][item[1]] = 1
            result[item[1]][item[0]] = 1
        for node_id in self.Graph.degree1:
            result[node_id][node_id] = 1
        return result
 
        
    def get_start_ids(self, ROBOT):
        start_ids = {}
        for robot in ROBOT:
            for i in range(self.node):
                if np.array_equal(self.Graph.graph.vs[i]['position'], robot):
                    if self.Graph.graph.vs[i]['id'] not in start_ids:
                        start_ids[self.Graph.graph.vs[i]['id']] = 1
                    else:
                        start_ids[self.Graph.graph.vs[i]['id']] += 1 
        self.start_ids = start_ids
    

    
    def iter_solution(self, out=(), i=1):  
        if len(self.solutions) > 0:
            return
        temp = list(out)
        self.checkConditions(temp)
        for j in range(self.case_num):
            self.iter_solution(out + (self.case_X_t[j], ), i + 1)
    
    def cal_combination_case_k_robot(self, k, out=()):
        # cal self.case_X_t [(k, node), (k, node)]
        if k == 0:
            temp = np.array(list(out))
            temp_case = np.reshape(temp, (self.k, self.node))
            self.case_X_t.append(temp_case)
            return
        for j in range(len(self.single_robot_X_t)):
            self.cal_combination_case_k_robot(k - 1, out + (self.single_robot_X_t[j], ))
        
    def iter_X_t(self):
        for i in range(self.node):
            result = np.array([0 for item in range(self.node)])
            result[i] = 1
            self.single_robot_X_t.append(result)
        
        
        


    def checkConditions(self, X):
        #  X :  [X0, X1, ..., XT]
        #  Xi:  np.shape(k, node)
        
        # robot must move to another place at X1, i.e. X1 != X0
        if np.max(X[0].dot(X[1].T)) >= 1:
            return
        
        # for any robot, it will appear on the end place.
        for i in range(self.k):
            summ = 0
            for item in self.Graph.degree1:
                summ += X[-1][i][item]
            if summ != 1:
                return
             
        # satisfy that Xi -> Xi+1 is reasonable
        for i in range(len(X) - 1):
            if np.max(X[i+1] > X[i].dot(self.A)):
                return
        
        # verify that all edges are routed
        weight = [0 for item in self.graph]        
        for i in range(len(X) - 1):
            for j in range(self.k):
                for index, item in enumerate(self.graph):
                    if X[i][j][item[0]] + X[i][j][item[1]] + X[i+1][j][item[0]] + X[i+1][j][item[1]] >= 2:
                        weight[index] = 1
        
        if min(weight) > 0:
            self.solutions.append(X)
            
        
        
    
    




            
    