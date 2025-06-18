import matplotlib.pyplot as plt
import datetime
import numpy as np
import timeit
import gurobipy as gp
from gurobipy import GRB

class BIPAlgorithm():
    def __init__(self, graph, robots):
        self.T_max = 16
        self.robots = robots
        self.Graph = graph
        self.num_nodes = self.Graph.node
        self.s0 = None
        self.s = None
        self.get_s0()
        self.num_robots = len(self.robots)
        self.edges = [list(elem) for elem in self.Graph.get_edges()]
        print(self.edges)
        self.num_edges = len(self.edges)
        self.A = self.get_adjance_matrix()
        
    def get_adjance_matrix(self):
        result = np.zeros((self.num_nodes, self.num_nodes))
        for item in self.edges:
            result[item[0]][item[1]] = 1
            result[item[1]][item[0]] = 1
        for node_id in self.Graph.degree1:
            result[node_id][node_id] = 1
        return result
 
        
    def get_s0(self):
        s0 = np.zeros((len(self.robots), self.num_nodes))
        flag = 0
        for robot in self.robots:
            for i in range(self.num_nodes):
                if np.array_equal(self.Graph.graph.vs[i]['position'], robot.position):
                    s0[flag, self.Graph.graph.vs[i]['id']] = 1
            flag += 1
        self.s0 = s0
        
    def solve(self):
        for t in range(1, self.T_max):
            opt = Optimizer(self.s0, self.edges, self.Graph.degree1, self.A, self.num_robots, self.num_edges, self.num_nodes, t)
            if(opt.solve()):
                self.s = opt.s
                break
        walks = self.trans(self.s0, self.s)
        # map1: cut_walks = [{'path': [0, 1, 2, 3, 5, 4, 2, 3, 5, 6, 7], 'length': 10, 'count': 11}]
        # map2: cut_walks = [{'path': [1, 0, 8, 9, 13, 9, 10, 11, 3, 2], 'length': 9, 'count': 10}, {'path': [7, 5, 15, 14, 10, 14, 13, 12, 4, 6], 'length': 9, 'count': 10}]
        # map3: cut_walks = [{'path': [1, 0, 8, 9, 13, 14, 13, 12, 4, 6], 'length': 9, 'count': 10}, {'path': [2, 3, 11, 10, 9, 10, 14, 15, 5, 7], 'length': 9, 'count': 10}]
        # map4: cut_walks = [{'path': [0, 1, 4, 5, 9], 'length': 4, 'count': 5}, {'path': [8, 7, 3, 2, 1, 0], 'length': 5, 'count': 6}, {'path': [9, 5, 6, 3, 7, 8], 'length': 5, 'count': 6}]
        # map5: cut_walks = [{'path': [0, 1, 2, 5, 2, 6], 'length': 5, 'count': 6}, {'path': [9, 4, 1, 3, 8], 'length': 4, 'count': 5}, {'path': [7, 3, 1, 4, 10], 'length': 4, 'count': 5}]
        # map6: cut_walks = [{'path': [7, 6, 0, 2, 0, 6, 7], 'length': 6, 'count': 7}, {'path': [9, 8, 1, 0, 1, 3, 15], 'length': 6, 'count': 7}, {'path': [11, 10, 4, 5, 4, 2, 14], 'length': 6, 'count': 7}, {'path': [13, 12, 5, 3, 2, 14], 'length': 5, 'count': 6}]
        unvisited_edge_num, sum_visited_edge = self.checkResult(walks)
        
        QQ = self.num_edges - unvisited_edge_num
        Repe =  (sum_visited_edge - QQ) / QQ
        
        return unvisited_edge_num, Repe, walks
    
    def checkResult(self, paths):
        temp_mat = -1 * np.ones((self.num_nodes, self.num_nodes))
        
        for edge in self.edges:
            temp_mat[edge[0]][edge[1]] = 0.
            temp_mat[edge[1]][edge[0]] = 0.

        for item in paths:
            path = item['path']
            for i in range(len(path)-1):
                if path[i] != path[i+1]:
                    temp_mat[path[i]][path[i+1]] += 1
                    temp_mat[path[i+1]][path[i]] += 1
        
        sum_visited_edge = 0
        unvisited_edge_num = 0
        for i in range(self.num_nodes):
            for j in range(i, self.num_nodes):
                if i != j:
                    if temp_mat[i][j] == 0:
                        unvisited_edge_num += 1
                    if temp_mat[i][j] > 0:
                        sum_visited_edge += temp_mat[i][j]

        return unvisited_edge_num, sum_visited_edge
    
    def trans(self, s0, s):
        # transform s0 and s to path
        paths = []
        for i in range(self.num_robots):
            path = [int(np.argmax(s0[i]))]
            for t in range(len(s)):
                temp = int(np.argmax(s[t, i]))
                if temp in self.Graph.degree1:
                    if temp == path[-1]:
                        continue
                path.append(temp)
        
            paths.append({'path': path, 'length': len(path)-1, 'count': len(path)})
            
        return paths


class Optimizer():
    def __init__(self, s_init, edges, node_degree_one, adjact_matrix, num_robots, edge_num, node_num, T):
        self.A = adjact_matrix
        self.Ne = edge_num
        self.m = node_num
        self.T = T
        self.num_robot = num_robots
        self.end_node_ids = node_degree_one
        self.edges = edges
        self.s0 = s_init
        self.b0_reshape = np.zeros((self.T, self.num_robot))
        self.b_shape = np.zeros((self.T, self.num_robot, self.Ne))
        
        self.s = np.zeros((self.T, self.num_robot, self.m))
        
        self.model = gp.Model("BIP")
        self.b = self.model.addVars(self.T, self.num_robot, self.Ne, vtype=GRB.BINARY, name="b")
        
        # ignore the state_0
        self.state = self.model.addVars(self.T, self.num_robot, self.m, vtype=GRB.BINARY, name="s")
        self.create_opt()
        
        
    def create_opt(self):
        # sum_m s_tli = 1
        for t in range(self.T):
            for l in range(self.num_robot):
                self.model.addConstr(gp.quicksum(self.state[t, l, n] for n in range(self.m)) == 1.)
                
        # s_t+1 <= s_t A
        for t in range(1, self.T):
            for l in range(self.num_robot):
                for n in range(self.m):
                    self.model.addConstr(gp.quicksum(self.state[t-1, l, k] * self.A[k, n] for k in range(self.m)) >= self.state[t, l, n])
        for l in range(self.num_robot):
            for n in range(self.m):
                self.model.addConstr(gp.quicksum(self.s0[l, k] * self.A[k, n] for k in range(self.m)) >= self.state[0, l, n])
        
        # sum_v s_Tli = 1
        for l in range(self.num_robot):
            self.model.addConstr(gp.quicksum(self.state[self.T-1, l, k] for k in self.end_node_ids) == 1.)
        
        # s0^T s1 = 0
        for l in range(self.num_robot):
            self.model.addConstr(gp.quicksum(self.s0[l, j] * self.state[0, l, j] for j in range(self.m)) == 0.)
        
        # any e_ij, sum_l sum_T b_tle_ij >= 1
        for e_id in range(self.Ne):
            self.model.addConstr(gp.quicksum(self.b[t, l, e_id] for t, l in np.ndindex(self.b0_reshape.shape)) >= 1.)
        
        # eq.(9)
        for t in range(0, self.T - 1):
            for l in range(self.num_robot):
                for e_id in range(self.Ne):
                    i, j = self.edges[e_id][0], self.edges[e_id][1]
                    self.model.addConstr(self.state[t, l, i] + self.state[t, l, j] + self.state[t+1, l, i] + self.state[t+1, l, j] >= 2 * self.b[t+1, l, e_id])
        for l in range(self.num_robot):
            for e_id in range(self.Ne):
                i, j = self.edges[e_id][0], self.edges[e_id][1]
                self.model.addConstr(self.s0[l, i] + self.s0[l, j] + self.state[0, l, i] + self.state[0, l, j] >= 2 * self.b[0, l, e_id])
                
        
        obj = gp.quicksum(self.b[i, j, k] for i, j, k in np.ndindex(self.b_shape.shape))
        self.model.setObjective(obj, GRB.MINIMIZE)
        
        
    def solve(self):
        starttime = timeit.default_timer()
        self.model.optimize()
        t_diff = timeit.default_timer() - starttime

        # print("*********************************************************")
        # print("Time to solve (ms)=",t_diff*1000)
        # print("*********************************************************")
        if self.model.status == GRB.Status.OPTIMAL:
            print('Optimal Solution found')
            for key, value in self.state.items():
                self.s[key] = self.state[key].x
            # for constr in self.model.getConstrs():
            #    print(self.model.getRow(constr), constr.RHS)
            return True
        elif self.model.status == GRB.Status.INF_OR_UNBD:
            print('Model is infeasible or unbounded')
            return False
        elif self.model.status == GRB.Status.INFEASIBLE:
            print('Model is infeasible')
            return False
        elif self.model.status == GRB.Status.UNBOUNDED:
            print('Model is unbounded')
            return False
        else:
            print('Optimization ended with status %d' % self.model.status)
            return False





if __name__ == "__main__":
    edges = [[0, 1], [1, 2], [2, 3], [3, 4], [1, 4]]
    degree1 = [0]
    num_robots = 1
    num_edges = 5
    num_nodes = 5
    s0 = np.zeros((num_robots, num_nodes))
    A = np.zeros((num_nodes, num_nodes))
    for item in edges:
        A[item[0], item[1]] = 1.
        A[item[1], item[0]] = 1.
    for item in degree1:
        A[item, item] = 1.
    s0[0, 0] = 1.
    begin = timeit.default_timer()
    for t in range(1, 9):
        opt = Optimizer(s0, edges, degree1, A, num_robots, num_edges, num_nodes, T=t)
        if(opt.solve()):
            delta_t = timeit.default_timer() - begin
            print(t, opt.s, delta_t)
            break