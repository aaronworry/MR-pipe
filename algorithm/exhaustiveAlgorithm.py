import time
import sys
import numpy as np

class ExhaustiveAlgorithm():
    def __init__(self, graph, ROBOT):        
        self.Graph = graph
        self.paths = []
        self.found = []
        self.get_start_ids(ROBOT)
        self.k = len(ROBOT)
        self.graph = [list(elem) for elem in self.Graph.get_edges()]

    def calculate_avaliable_solution(self):
        for edge in self.graph:
            for node in edge:
                if node in self.start_ids:
                    self.findPaths([node])
        self.findCombinations(self.paths, self.k)
        return self.found
        
    def find_optimize_solution(self):
        walks = None
        minLen = sys.maxsize
        for e in self.found:
            lenList = []
            simple_closed_walk = []
            for walk in e:
                len1 = len(walk) - 1
                lenList.append(len1)
                simple_closed_walk.append({'path': walk, 'length': len1, 'count': len(walk)})
            tempmax = max(lenList)
            if tempmax < minLen:
                walks = simple_closed_walk
                minLen = tempmax
        return walks
        
    def get_start_ids(self, ROBOT):
        start_ids = {}
        for robot in ROBOT:
            for i in range(self.Graph.node):
                if np.array_equal(self.Graph.graph.vs[i]['position'], robot):
                    if self.Graph.graph.vs[i]['id'] not in start_ids:
                        start_ids[self.Graph.graph.vs[i]['id']] = 1
                    else:
                        start_ids[self.Graph.graph.vs[i]['id']] += 1 
        self.start_ids = start_ids

    def get_adj(self, node):
        new_edges = []
        for edge in self.graph:
            if node in edge:
                temp = [edge[0], edge[1]]
                temp.remove(node)
                new_edges.append(temp[0])
        return new_edges
        
    def findPaths(self, path):
        # path 路径的倒序
        start_node = path[0]
        next_node = None
        sub = []
        for edge in self.graph:
            node1, node2 = edge
            if start_node in edge:
                if node1 == start_node:
                    next_node = node2
                else:
                    next_node = node1
                    
                if not self.visited(next_node, path):
                    sub = [next_node]
                    sub.extend(path)
                    if self.Graph.graph.degree(sub[0]) == 1:
                        inv = sub[::-1]  # 调整路径的顺序
                        if self.isNew(inv):
                            self.paths.append(inv)
                    self.findPaths(sub)                    
                elif next_node != path[1]: # 不走回头路 
                    # next_node 的所有邻接节点是否经过
                    adj_list = self.get_adj(next_node)
                    node_flag = False
                    for node in adj_list:
                        if not self.visited(node, path):
                            sub = [node, next_node]
                            sub.extend(path)
                            if self.Graph.graph.degree(sub[0]) == 1:
                                inv = sub[::-1]
                                if self.isNew(inv):
                                    self.paths.append(inv)
                            self.findPaths(sub)
                        
                    

    def isNew(self, path):    # 判断路径是否存在
        return not path in self.paths

    def visited(self, node, path):  # 节点是否在路径中
        return node in path

    def findMatch(self, paths):
        temp = self.start_ids.copy()
        for path in paths:
            if path[0] in temp:
                temp[path[0]] -= 1
        for item in temp:
            if temp[item] != 0:
                return False
        if self.checkConditions(paths):
            self.found.append(paths)
            return True
        else:
            return False

    def checkConditions(self, paths):
        lst = [0] * len(self.graph)
        for path in paths:
            i = 0
            for edge in self.graph:
                if self.check_added2(path, edge):
                    lst[i] = 1
                i = i + 1
        if 0 in lst:
            return False
        return True

    def check_added2(self, path, edge):    # 边是否在 路径中
        if self.sub_list_exists(path, edge):
            return True
        temp_edge = [edge[1], edge[0]]
        if self.sub_list_exists(path, temp_edge):
            return True
        return False

    def sub_list_exists(self, list1, list2):           # list2 是否 是 list1 的一部分
        if len(list2) < 2:
            return False
        return ''.join(map(str, list2)) in ''.join(map(str, list1))

    def findCombinations(self, A, k, out=(), i=0):
        # k <= len(A) - i
        if len(A) == 0 or k > len(A):
            return
        if k == 0:
            self.findMatch(out)
            return
        for j in range(i, len(A)):
            # add current element `A[j]` to the solution and recur for next index
            # `j+1` with one less element `k-1`
            
            self.findCombinations(A, k - 1, out + (A[j],), j + 1)
    



if __name__ == '__main__':
    sys.path.append("..")
    from env import Env
    from .weightGraph import *
    
    vertices0 = np.array([[0, 0, 0], [0, -1, 0], [0, -2, 0], [0, -3, 0], [1, 0, 0], [1, -3, 0], [3, 0, 0], [3, -3, 0]])
    vertices1 = np.array([[0, 0, 1], [0, -1, 1], [0, -2, 1], [0, -3, 1], [1, 0, 1], [1, -1, 1], [1, -2, 1], [1, -3, 1]])
    
    Edge0 = np.asarray([np.asarray([vertices0[0], vertices0[1]]), np.asarray([vertices0[2], vertices0[3]]), np.asarray([vertices0[4], vertices0[6]]), np.asarray([vertices0[5], vertices0[7]])])
    Edge1 = np.asarray([np.asarray([vertices1[0], vertices1[1]]), np.asarray([vertices1[1], vertices1[2]]), np.asarray([vertices1[2], vertices1[3]]), np.asarray([vertices1[4], vertices1[5]]), np.asarray([vertices1[5], vertices1[6]]), np.asarray([vertices1[6], vertices1[7]]), np.asarray([vertices1[1], vertices1[5]]), np.asarray([vertices1[2], vertices1[6]])])
    Edge0_1 = np.asarray([np.asarray([vertices0[0], vertices1[0]]), np.asarray([vertices0[3], vertices1[3]]), np.asarray([vertices0[4], vertices1[4]]), np.asarray([vertices0[5], vertices1[7]])])

    pipeDict2 = {'vertices': [vertices0, vertices1], 'verticalEdge': [Edge0_1], 'horizonEdge': [Edge0, Edge1], 'Edge': [Edge0, Edge1, Edge0_1]}
    ROBOT = [np.array([0, -1, 0]), np.array([0, -2, 0])]

    env = Env(dt = 2, pipeDict = pipeDict2)
    weightGraph = WeightGraph(env.vertices, env.Pipe)
    # weightGraph.generate_graph(env.vertices, env.Pipe)
    """
    graph = weightGraph.graph
    
    start_ids = {}
    for robot in ROBOT:
        for i in range(weightGraph.node):
            if np.array_equal(graph.vs[i]['position'], robot):
                if graph.vs[i]['id'] not in start_ids:
                    start_ids[graph.vs[i]['id']] = 1
                else:
                    start_ids[graph.vs[i]['id']] += 1  
    """
    
    alg = MySimpleAlgorithm(weightGraph, ROBOT)
    alg.calculate_avaliable_solution()
    
    """
    walks = None
    minLen = sys.maxsize
    for e in found:
        lenList = []
        simple_closed_walk = []
        for walk in e:
            len1 = len(walk) - 1
            lenList.append(len1)
            simple_closed_walk.append({'path': walk, 'length': len1, 'count': len(walk)})
        tempmax = max(lenList)
        if tempmax < minLen:
            walks = simple_closed_walk
            minLen = tempmax
    """
    walks = alg.find_optimize_solution()
    print(walks)
    
    position_paths = []
    for item in walks:
        # item['path'] = []
        edge_list = []
        temp = item['path']
        for i in range(len(temp)-1):
            id1, id2 = temp[i], temp[i+1]            
            pos1, pos2 = None, None
            for j in range(weightGraph.node):
                if weightGraph.graph.vs[j]['id'] == id1:
                    pos1 = weightGraph.graph.vs[j]['position']
                elif weightGraph.graph.vs[j]['id'] == id2:
                    pos2 = weightGraph.graph.vs[j]['position']
                if pos1 is not None and pos2 is not None:
                    edge = [pos1, pos2]
                    edge_list.append(edge)
                    break
        position_paths.append(edge_list)
        
    print(position_paths)
        
        
            
    