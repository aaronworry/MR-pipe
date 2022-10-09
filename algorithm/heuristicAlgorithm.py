from operator import itemgetter
import time
import sys
import numpy as np


# I didn't want to bother with global variables therefore,
# I created a class to encapsulate my algorithm.
class HeuristicAlgorithm():

    def __init__(self, graph, ROBOT):
        self.Graph = graph
        self.__edges = None
        self.__sorted_edges = []
        self.paths = {}
        self.get_start_ids(ROBOT)
        self.k = len(ROBOT)
        self.graph = [list(elem) for elem in self.Graph.simpleGraph.get_edgelist()]
        
    def get_start_ids(self, ROBOT):
        start_ids = {}
        for robot in ROBOT:
            for i in range(self.Graph.node):
                if np.array_equal(self.Graph.graph.vs[i]['position'], robot):
                    if self.Graph.graph.vs[i]['id'] not in start_ids:
                        start_ids[self.Graph.graph.vs[i]['id']] = 1
                        self.paths[self.Graph.graph.vs[i]['id']] = []
                    else:
                        start_ids[self.Graph.graph.vs[i]['id']] += 1 
        self.start_ids = start_ids

    def my_algorithm(self):
        self.sort_edges_descending()
        self.create_paths()
        return self.paths

    def sort_edges_descending(self):
        weights = self.Graph.simpleGraph.es['weight']
        edges = self.Graph.simpleGraph.get_edgelist()
        self.__edges = edges
        n = len(weights)
        for i in range(n):
            for j in range(0, n - i - 1):
                if weights[j] < weights[j + 1]:
                    edges[j], edges[j + 1] = edges[j + 1], edges[j]
                    weights[j], weights[j + 1] = weights[j + 1], weights[j]
        self.create_edge_dict(edges, weights)

    def create_edge_dict(self, edges, weights):
        n = len(weights)
        for i in range(n):
            edge = edges[i]
            my_dict = {
                'start_node': edge[0],
                'end_node': edge[1],
                'length': weights[i]
            }
            self.__sorted_edges.append(my_dict)

    def get_edge_length(self, edge):
        for e in self.__sorted_edges:
            if (e['start_node'] == edge[0] and e['end_node'] == edge[1]) or \
                    (e['start_node'] == edge[1] and e['end_node'] == edge[0]):
                return e['length']
        return 0

    def is_in_edge_list(self, edge):
        for e in self.__sorted_edges:
            if (e['start_node'] == edge[0] and e['end_node'] == edge[1]) or \
                    (e['start_node'] == edge[1] and e['end_node'] == edge[0]):
                return True
        return False

    def create_paths(self):
        for e in self.__sorted_edges:
            path_temp = [e['start_node'], e['end_node']]
            for item in self.start_ids:
                walk = []
                if not self.check_added(path_temp, self.paths[item]):
                    path1 = self.Graph.simpleGraph.get_shortest_paths(item, to=path_temp[0], weights=self.Graph.simpleGraph.es["weight"], output="vpath",)[0]
                    for goal in self.Graph.degree1:
                        path2 = None
                        distance = sys.maxsize
                        if goal != item:
                            path = self.Graph.simpleGraph.get_shortest_paths(path_temp[1], to=goal, weights=self.Graph.simpleGraph.es["weight"], output="vpath",)[0]
                            temp = self.Graph.get_length_simple_graph(path)
                            if temp < distance:
                                path2 = path
                                distance = temp
                    self.try_to_merge(path1, path2, walk)   
                    self.paths[item].append(
                        {'path': walk, 'length': self.get_walk_length(walk), 'count': len(walk)})
        for item in self.paths:
            self.paths[item] = sorted(self.paths[item], key=itemgetter('length'), reverse=True)   # 降序
        for i in self.start_ids:
            if self.start_ids[i] > len(self.paths[i]):
                self.add_dummy_tours(i, self.start_ids[i] - len(self.paths[i]))
        result = self.merge_walks()   # [{}, {}, {}, ...]        
        self.paths = result

    def check_added(self, edge, paths):
        for e in paths:
            walk = e['path']
            if self.sub_list_exists(walk, edge):
                return True
            reverse_edge = [edge[1], edge[0]]
            if self.sub_list_exists(walk, reverse_edge):
                return True
        return False

    def add_dummy_tours(self, idx, missing_number):
        for j in range(missing_number):
            self.paths[idx].append({'path': [idx], 'length': 0, 'count': 1})

    
    def merge_walks(self):
        result = self.find_combinations()
        
        R, maxL, totalL = None, sys.maxsize, sys.maxsize
        
        for path_comb in result:
            edge_assigned_list = self.assign_edges(path_comb)
            # 统计未经过的edge，将edge分配给这些组合，edge离哪条路径最近，就分配给哪条, 并由小到大排序
            updated_path = self.update_paths(path_comb, edge_assigned_list)
            # 对于每个路径，按被分配edge离路径的距离从小到大的顺序添加edge到该路径中，添加一个edge, 则更新一次路径
            
            # 对于每条路径，记录里面的重复的子路径，找到最短的子路径进行替换
            total_length, max_length = 0, 0
            for path in updated_path:
                total_length += path['length']
                if max_length <= path['length']:
                    max_length = path['length']
            # 计算路径总长度以及最大路径长度
            if maxL > max_length:
                maxL = max_length
                R = updated_path
            # 选最大的长度最小的
            return R

    
    def update_paths(self, path_comb, edge_assigned_list):
        # path_comb [{}, {}, {}]   edge_assigned_list    [[{}, {}, {}], [{}, {}, {}], [{}, {}, {}]]
        n = len(path_comb)
        for i in range(n):
            initial_path = path_comb[i]['path'].copy()
            for edge in edge_assigned_list[i]:   # {}
                # edge['edge'] = [a, b]        edge['edge_id'] = a 或 b      edge['id'] = q
                _, edge_temp, id_temp = self.cal_edge_path_distance(initial_path, edge)
                insert_path = self.Graph.simpleGraph.get_shortest_paths(id_temp, to=edge_temp, weights=self.Graph.simpleGraph.es["weight"], output="vpath",)[0]
                path_temp2 = insert_path.copy()
                path_temp2.reverse()
                path_one = edge['edge'][0] if edge['edge'][1] == edge_temp else edge['edge'][1]
                insert_path.extend([path_one])
                insert_path.extend([path_temp2])
                for i, idx in enumerate(initial_path):
                    if idx == edge['id']:
                        a = initial_path[:i]
                        b = initial_path[i+1:]
                        a.extend(insert_path)
                        a.extend(b)
                        initial_path = a
                        break
            path_comb[i]['path'] = initial_path
            path_comb[i]['length'] = self.get_walk_length(initial_path)
            path_comb[i]['count'] = len(initial_path)
        return path_comb

                    

    def assign_edges(self, path_comb):
        # [{}, {}, {}]
        edge_dict = {}
        unreached_edge = []    # 未经过的边
        for item in path_comb:
            path = item['path']
            for i in range(len(path)-1):
                if (path[i], path[i+1]) in edge_dict:
                    edge_dict[(path[i], path[i+1])] += 1
                elif (path[i+1], path[i]) in edge_dict:
                    edge_dict[(path[i+1], path[i])] += 1
                else:
                    edge_dict[(path[i], path[i+1])] = 1
        for item in self.graph:
            if tuple((item[0], item[1])) in edge_dict or tuple((item[1], item[0])) in edge_dict:
                continue
            else:
                unreached_edge.append(item)
        
        assign_edges = [[]] * len(path_comb)              #  [  [{}, {}, {}],   [],  [] ]
        for edge in unreached_edge:
            distance, edge_flag, id_flag = sys.maxsize, None, 0
            assign_id = 0
            for i in range(len(path_comb)):
                dist_temp, edge_temp, id_temp = self.cal_edge_path_distance(path_comb[i]['path'], edge)
                if dist_temp < distance:
                    distance, edge_flag, id_flag = dist_temp, edge_temp, id_temp
                    assign_id = i
            assign_edges[assign_id].append({'edge':edge, 'dis':distance, 'edge_id':edge_flag, 'id':id_temp})
        for i in range(len(assign_edges)):
            assign_edges[i] = sorted(assign_edges[i], key=itemgetter('dis'))
        return assign_edges
        
    def cal_edge_path_distance(self, path, edge):
        if edge[0] in path:
            return 0, edge[0], edge[0]
        elif edge[1] in path:
            return 0, edge[1], edge[1]
        else:
            result = sys.maxsize
            id_temp = 0
            edge_temp = None
            for idx in path:
                path_temp1 = self.Graph.simpleGraph.get_shortest_paths(idx, to=edge[0], weights=self.Graph.simpleGraph.es["weight"], output="vpath",)[0]
                path_temp2 = self.Graph.simpleGraph.get_shortest_paths(idx, to=edge[1], weights=self.Graph.simpleGraph.es["weight"], output="vpath",)[0]
                dist1 = self.Graph.get_length_simple_graph(path_temp1)
                dist2 = self.Graph.get_length_simple_graph(path_temp2)
                if min(dist1, dist2) < result:
                    id_temp = idx
                    if dist1 < dist2:
                        result = dist1
                        edge_temp = edge[0]
                    else:    
                        result = dist2
                        edge_temp = edge[1]
            return result, edge_temp, id_temp
                

    def find_combinations(self):
        result = []
        aspect_result = []
        for item in self.start_ids:
            aspect_temp = []
            print(self.paths[item], self.start_ids[item])
            print("--------")
            self.aspect_comb(self.paths[item], self.start_ids[item], aspect_temp, [], 0)
            aspect_result.append(aspect_temp)          #aspect_temp:  [   [{}, {}]    [{}, {}]       ]
        self.global_comb(aspect_result, [], 0)
        # result = [ [{}, {}, {}],  [{}, {}, {}],   [{}, {}, {}]      ]
        return result
    
    def global_comb(self, A, out=[], i=0):        
        if i >= len(A):
            result.append(out)
            return        
        for j in range(len(A[i])):
            self.global_comb(A, out.extend(A[i][j]), i + 1)
        
    def aspect_comb(self, A, k, aspect_temp, out=[], i=0):
        # k <= len(A) - i
        if len(A) == 0 or k > len(A):
            return
        if k == 0:
            aspect_temp.append(out)
            return
        for j in range(i, len(A)):
            # add current element `A[j]` to the solution and recur for next index
            # `j+1` with one less element `k-1`
            self.aspect_comb(A, k - 1, aspect_temp, out.extend([A[j]]), j + 1)

    def sub_list_exists(self, list1, list2):
        if len(list2) < 2:
            return False
        return ''.join(map(str, list2)) in ''.join(map(str, list1))

    def try_to_merge(self, path1, path2, walk):
        if len(path1) == 1 or len(path2) == 1:
            walk.extend(path1)
            walk.extend(path2)
        else:
            if path1[-2] == path2[0] and path1[-1] == path2[1]:
                walk.extend(path1)
                if len(path2) > 2:
                    walk.extend(path2[2:])
            else:
                walk.extend(path1)
                walk.extend(path2)

    def get_walk_length(self, walk):
        if len(walk) > 0:
            # Add up the weights across all edges on the shortest path
            distance = 0
            n = len(walk)
            for i in range(0, n):
                if i + 1 != n:
                    edge = [walk[i], walk[i + 1]]
                    distance += self.get_edge_length(edge)
            return distance
        else:
            print("walk is empty")
            return 0

