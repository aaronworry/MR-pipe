import time
import sys
import numpy as np
# m node, n robot, T length        (m^n)^T
sys.setrecursionlimit(10000)

# cost so many time
class ExhaustiveSpaceToTime():
    def __init__(self, graph, robots):     
        self.robots = robots
        self.Graph = graph
        self.node = self.Graph.node
        self.start_ids = None
        self.get_start_ids(self.robots)
        self.k = len(self.robots)
        self.graph = [list(elem) for elem in self.Graph.get_edges()]
        
        self.single_robot_X_t = []
        self.case_X_t = []
        self.case_num = 0
        self.solutions = []
        self.X0 = None
        
        self.iter_X_t()
        self.A, self.robot_node_dict = self.get_adjance_matrix()
        
    def solve(self):
        self.case_num = pow(len(self.single_robot_X_t), self.k)
        self.cal_combination_case_k_robot(self.k)
        # cal X0 based on initial state
        X0 = np.zeros((self.k, self.node))
        flag = 0
        for key, value in self.start_ids.items():
            j = value
            while j > 0:
                X0[flag][int(key)] = 1
                j -= 1
                flag += 1
        self.X0 = X0
        self.iter_solution(self.case_X_t, self.X0, (self.X0, ), 1)
        return self.trans(self.solutions[0])
    
    
    def trans(self, X):
        # X to path
        paths = []
        for i in range(self.k):
            path = []
            for j in range(len(X)):
                temp = np.argmax(X[j][i])
                if temp in self.Graph.degree1 and len(path) > 0:
                    if temp == path[-1]:
                        break
                path.append(temp)
        
            paths.append({'path': path})
            
        return paths
        
    
    
    def get_adjance_matrix(self):
        result = np.zeros((self.node, self.node))
        robot_nodes = {}                # {id:[np.zeros], }
        for item in self.graph:
            result[item[0]][item[1]] = 1
            result[item[1]][item[0]] = 1
            
            if item[0] not in robot_nodes:
                robot_nodes[item[0]] = [self.single_robot_X_t[item[1]]]
            else:
                robot_nodes[item[0]].append(self.single_robot_X_t[item[1]])
            
            if item[1] not in robot_nodes:
                robot_nodes[item[1]] = [self.single_robot_X_t[item[0]]]
            else:
                robot_nodes[item[1]].append(self.single_robot_X_t[item[0]])
            
        for node_id in self.Graph.degree1:
            result[node_id][node_id] = 1
            robot_nodes[node_id].append(self.single_robot_X_t[node_id])
        return result, robot_nodes
 
        
    def get_start_ids(self, robots):
        start_ids = {}
        for robot in robots:
            for i in range(self.node):
                if np.array_equal(self.Graph.graph.vs[i]['position'], robot.position):
                    if self.Graph.graph.vs[i]['id'] not in start_ids:
                        start_ids[self.Graph.graph.vs[i]['id']] = 1
                    else:
                        start_ids[self.Graph.graph.vs[i]['id']] += 1 
        self.start_ids = start_ids
    

    
    def iter_solution(self, X_list, X0, out=(), iter_num=0):
        if iter_num > 20:
            return
        if len(self.solutions) > 0:
            return 
        Case = self.checkConditions(out)
        
        for item_case in X_list:
            if iter_num == 1:
                # robot must move to another place at X1, i.e. X1 != X0
                if np.max(X0.dot(item_case.T)) < 0.9 and np.min(item_case <= X0.dot(self.A)) == 1:
                    self.iter_solution(X_list, X0, out + (item_case, ), iter_num + 1)
            else:
                temp = list(out)
                # satisfy that Xi -> Xi+1 is reasonable
                if np.min(item_case <= temp[-1].dot(self.A)) == 1:
                    self.iter_solution(X_list, X0, out + (item_case, ), iter_num + 1)
    
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
        
        

    def checkConditions(self, x):
        X = list(x)
        #  X :  [X0, X1, ..., XT]
        #  Xi:  np.shape(k, node)
        if len(X) < 2:
            return False
        # for any robot, it will appear on the end place.
        for i in range(self.k):
            print(np.argmax(X[-1][i]))
            if np.argmax(X[-1][i]) not in self.Graph.degree1:
                return False

        # verify that all edges are routed
        weight = [0 for item in self.graph]        
        for i in range(len(X) - 1):
            for j in range(self.k):
                for index, item in enumerate(self.graph):
                    if X[i][j][item[0]] + X[i][j][item[1]] + X[i+1][j][item[0]] + X[i+1][j][item[1]] >= 2:
                        weight[index] = 1
        
        if min(weight) > 0:
            self.solutions.append(X)
            return True
        return False
            
        
        
    
    




            
    