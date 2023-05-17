import time
import sys
import numpy as np

class DFSAlgorithm():
    def __init__(self, graph, robots):
        self.robots = robots
        self.Graph = graph
        self.paths = []
        self.found = []
        self.get_start_ids(self.robots)
        self.k = len(self.robots)
        self.gamma = 100.
        self.graph = [list(elem) for elem in self.Graph.get_edges()]

    def calculate_avaliable_solution(self):
        for edge in self.graph:
            for node in edge:
                if node in self.start_ids:
                    self.findPaths([node])
        self.findCombinations(self.paths, self.k)
        
    def find_optimize_solution(self):
        self.calculate_avaliable_solution()
        walks = None
        minLen = sys.maxsize
        u_num = 0
        all_num = 0
        for num, All, e in self.found:
            lenList = []
            simple_closed_walk = []
            for walk in e:
                len1 = len(walk) - 1
                lenList.append(len1)
                simple_closed_walk.append({'path': walk, 'length': len1, 'count': len(walk)})
            tempmax = max(lenList) + self.gamma * num
            if tempmax < minLen:
                walks = simple_closed_walk
                minLen = tempmax
                u_num = num
                all_num = All
        QQ = len(self.graph) - u_num
        Repe =  (all_num - QQ) / QQ
        return u_num, Repe, walks
        
    def get_start_ids(self, robots):
        start_ids = {}
        for robot in robots:
            for i in range(self.Graph.node):
                if np.array_equal(self.Graph.graph.vs[i]['position'], robot.position):
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
        # reverse order of the path
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
                        inv = sub[::-1]  # reverse the path
                        if self.isNew(inv):
                            self.paths.append(inv)
                    self.findPaths(sub)                    
                elif next_node != path[1]: # not move back 
                    # all adjacent nodes of the next_node are visited?
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
                        
                    

    def isNew(self, path):    # Path exist?
        return not path in self.paths

    def visited(self, node, path):  # Node in Path?
        return node in path

    def findMatch(self, paths):
        temp = self.start_ids.copy()
        for path in paths:
            if path[0] in temp:
                temp[path[0]] -= 1
        for item in temp:
            if temp[item] != 0:
                return False
        unvisited_edge_num, sum_visited_edge = self.checkConditions(paths)
        self.found.append([unvisited_edge_num, sum_visited_edge, paths])
        """
        if self.checkConditions(paths):
            self.found.append(paths)
            return True
        else:
            return False
        """

    def checkConditions(self, paths):
        lst = [0] * len(self.graph)
        result = 0
        for path in paths:
            i = 0
            for edge in self.graph:
                if self.check_added2(path, edge):
                    lst[i] += 1
                i = i + 1
        for item in lst:
            if item == 0:
                result += 1
        all_pass = sum(lst)
        return result, all_pass

    def check_added2(self, path, edge):    # whether the Edge is in Path
        if self.sub_list_exists(path, edge):
            return True
        temp_edge = [edge[1], edge[0]]
        if self.sub_list_exists(path, temp_edge):
            return True
        return False

    def sub_list_exists(self, list1, list2):           # whether list2 is part of list1
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
    
            
    