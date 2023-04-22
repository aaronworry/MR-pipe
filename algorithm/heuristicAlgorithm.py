from operator import itemgetter
import time
import sys
import numpy as np


# I didn't want to bother with global variables therefore,
# I created a class to encapsulate my algorithm.
class HeuristicAlgorithm():

    def __init__(self, graph, robots):
        self.robots = robots
        self.Graph = graph
        self.__edges = None
        self.__sorted_edges = []
        self.found = []
        self.update_found = []
        self.aspect_found = {}
        self.paths = {}
        self.result = []
        self.get_start_ids(self.robots)
        self.k = len(self.robots)
        self.graph = [list(elem) for elem in self.Graph.get_edges()]
        
    def get_start_ids(self, robots):
        start_ids = {}
        for robot in robots:
            for i in range(self.Graph.node):
                if np.array_equal(self.Graph.graph.vs[i]['position'], robot.position):
                    if self.Graph.graph.vs[i]['id'] not in start_ids:
                        start_ids[self.Graph.graph.vs[i]['id']] = 1
                        self.paths[self.Graph.graph.vs[i]['id']] = []
                        self.aspect_found[self.Graph.graph.vs[i]['id']] = []
                    else:
                        start_ids[self.Graph.graph.vs[i]['id']] += 1 
        self.start_ids = start_ids

    def my_algorithm(self):
        self.sort_edges_descending()
        self.create_paths()
        
        unvisited_edge_num, sum_visited_edge = self.checkResult(self.result)
        
        QQ = len(self.graph) - unvisited_edge_num
        Repe =  (sum_visited_edge - QQ) / QQ
        
        return unvisited_edge_num, Repe, self.result
    
    def checkResult(self, paths):
        temp_mat = np.zeros((self.Graph.node, self.Graph.node))
        for item in paths:
            path = item['path']
            for i in range(len(path)-1):
                if path[i] != path[i+1]:
                    temp_mat[path[i]][path[i+1]] += 1
                    temp_mat[path[i+1]][path[i]] += 1
        sum_visited_edge = np.sum(temp_mat) / 2
        return 0, sum_visited_edge
        
    
        lst = [0] * len(self.graph)
        result = 0
        for item in paths:
            path = item['path']
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

    def sort_edges_descending(self):
        weights = self.Graph.graph.es['weight']
        edges = self.Graph.graph.get_edgelist()
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
        """
        for e in self.__sorted_edges:
            path_temp = [e['start_node'], e['end_node']]
            for item in self.start_ids:
                walk = []
                if not self.check_added(path_temp, self.paths[item]):
                    path1 = self.Graph.graph.get_shortest_paths(item, to=path_temp[0], weights=self.Graph.graph.es["weight"], output="vpath",)[0]
                    path2 = None
                    distance = sys.maxsize
                    for goal in self.Graph.degree1:
                        if goal != item:
                            path = self.Graph.graph.get_shortest_paths(path_temp[1], to=goal, weights=self.Graph.graph.es["weight"], output="vpath",)[0]
                            temp = self.Graph.get_length_graph(path)
                            if temp < distance:
                                path2 = path
                                distance = temp
                    self.try_to_merge(path1, path2, walk)   
                    self.paths[item].append({'path': walk, 'length': self.get_walk_length(walk), 'count': len(walk)})
        """
        
        for e in self.__sorted_edges:
            path_temp = [e['start_node'], e['end_node']]
            if not self.check_added_all(path_temp, self.paths):
                temp_item, temp_walk = 0, {'path': None, 'length': sys.maxsize, 'count': 0}
                for item in self.start_ids:
                    walk = []
                    path1 = self.Graph.graph.get_shortest_paths(item, to=path_temp[0], weights=self.Graph.graph.es["weight"], output="vpath",)[0]
                    path2, distance = None, sys.maxsize
                    for goal in self.Graph.degree1:
                        path = self.Graph.graph.get_shortest_paths(path_temp[1], to=goal, weights=self.Graph.graph.es["weight"], output="vpath",)[0]
                        temp = self.Graph.get_length_graph(path)
                        if temp <= distance:
                            path2 = path
                            distance = temp
                    self.try_to_merge(path1, path2, walk)
                    path_temp1 = {'path': walk, 'length': self.get_walk_length(walk), 'count': len(walk)}
                    
                    walk = []
                    path1 = self.Graph.graph.get_shortest_paths(item, to=path_temp[1], weights=self.Graph.graph.es["weight"], output="vpath",)[0]
                    path2, distance = None, sys.maxsize
                    for goal in self.Graph.degree1:
                        path = self.Graph.graph.get_shortest_paths(path_temp[0], to=goal, weights=self.Graph.graph.es["weight"], output="vpath",)[0]
                        temp = self.Graph.get_length_graph(path)
                        if temp <= distance:
                            path2 = path
                            distance = temp
                    self.try_to_merge(path1, path2, walk)
                    path_temp2 = {'path': walk, 'length': self.get_walk_length(walk), 'count': len(walk)}
                    
                    if path_temp1['length'] <= path_temp2['length']:
                        if path_temp1['length'] < temp_walk['length']:
                            temp_item, temp_walk = item, path_temp1
                    else:
                        if path_temp2['length'] < temp_walk['length']:
                            temp_item, temp_walk = item, path_temp2
                self.paths[temp_item].append(temp_walk)
        """
        for item in self.start_ids:
            for e in self.__sorted_edges:
                path_temp = [e['start_node'], e['end_node']]
                if not self.check_added(path_temp, self.paths[item]):
                    walk = []
                    path1 = self.Graph.graph.get_shortest_paths(item, to=path_temp[0], weights=self.Graph.graph.es["weight"], output="vpath",)[0]
                    path2, distance = None, sys.maxsize
                    for goal in self.Graph.degree1:
                        path = self.Graph.graph.get_shortest_paths(path_temp[1], to=goal, weights=self.Graph.graph.es["weight"], output="vpath",)[0]
                        temp = self.Graph.get_length_graph(path)
                        if temp <= distance:
                            path2 = path
                            distance = temp
                    self.try_to_merge(path1, path2, walk)
                    path_temp1 = {'path': walk, 'length': self.get_walk_length(walk), 'count': len(walk)}
                    
                    walk = []
                    path1 = self.Graph.graph.get_shortest_paths(item, to=path_temp[1], weights=self.Graph.graph.es["weight"], output="vpath",)[0]
                    path2, distance = None, sys.maxsize
                    for goal in self.Graph.degree1:
                        path = self.Graph.graph.get_shortest_paths(path_temp[0], to=goal, weights=self.Graph.graph.es["weight"], output="vpath",)[0]
                        temp = self.Graph.get_length_graph(path)
                        if temp <= distance:
                            path2 = path
                            distance = temp
                    self.try_to_merge(path1, path2, walk)
                    path_temp2 = {'path': walk, 'length': self.get_walk_length(walk), 'count': len(walk)}
                    
                    
                    if path_temp1['length'] <= path_temp2['length']:
                        self.paths[item].append(path_temp1)
                    else:
                        self.paths[item].append(path_temp2)
        """
        for item in self.paths:
            self.paths[item] = sorted(self.paths[item], key=itemgetter('length'), reverse=True)   # 降序
        for i in self.start_ids:
            if self.start_ids[i] > len(self.paths[i]):
                self.add_dummy_tours(i, self.start_ids[i] - len(self.paths[i]))
        result = self.merge_walks()   # [{}, {}, {}, ...] 
        for item in result:
            if item['count'] == 1:
                item['count'] = 3
                item['length'] = 2
                temp_path_node = item['path'][0]
                new_node = self.Graph.get_adj(temp_path_node, self.Graph.get_edges())[0]
                item['path'].append(new_node)
                item['path'].append(temp_path_node)
        self.result = result

    def check_added(self, edge, paths):
        for e in paths:
            walk = e['path']
            if self.sub_list_exists(walk, edge):
                return True
            reverse_edge = [edge[1], edge[0]]
            if self.sub_list_exists(walk, reverse_edge):
                return True
        return False
    
    def check_added_all(self, edge, paths_dict):
        for item in self.start_ids:
            for e in paths_dict[item]:
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
        self.find_combinations()
        R_id, maxL, totalL = None, sys.maxsize, sys.maxsize
        for index, path_comb in enumerate(self.found):
            edge_assigned_list = self.assign_edges(path_comb)
            # 统计未经过的edge，将edge分配给这些组合，edge离哪条路径最近，就分配给哪条, 并由小到大排序
            self.update_paths(path_comb, edge_assigned_list)
            # 对于每个路径，按被分配edge离路径的距离从小到大的顺序添加edge到该路径中，添加一个edge, 则更新一次路径
            
            # 对于每条路径，记录里面的重复的子路径，找到最短的子路径进行替换
        # print("++++", self.update_found)
        for index, path_list in enumerate(self.update_found):
            total_length, max_length = 0, 0
            for path in path_list:
                total_length += path['length']
                if max_length <= path['length']:
                    max_length = path['length']
            # 计算路径总长度以及最大路径长度
            if max_length <= maxL:
                maxL = max_length
                R_id = index
            # 选最大的长度最小的
        # print(self.update_found[R_id])
        return self.update_found[R_id]

    
    def update_paths(self, path_comb, edge_assigned_list):
        # path_comb [{}, {}, {}]   edge_assigned_list    [[{}, {}, {}], [{}, {}, {}], [{}, {}, {}]]
        update_path_list = []
        for i, value in enumerate(path_comb):
            initial_path = value['path'].copy()
            for edge in edge_assigned_list[i]:   # {}
                # edge['edge'] = [a, b]        edge['edge_id'] = a 或 b      edge['id'] = q
                dist_temp, edge_temp, id_temp = self.cal_edge_path_distance(initial_path, edge['edge'])
                insert_path = self.Graph.graph.get_shortest_paths(id_temp, to=edge_temp, weights=self.Graph.graph.es["weight"], output="vpath",)[0]
                path_temp2 = insert_path.copy()
                path_temp2.reverse()
                path_one = edge['edge'][0] if edge['edge'][1] == edge_temp else edge['edge'][1]
                insert_path.extend([path_one])
                insert_path.extend(path_temp2)
                for index, idx in enumerate(initial_path):
                    if idx == id_temp:
                        a = initial_path[:index]
                        b = initial_path[index+1:]
                        a.extend(insert_path)
                        a.extend(b)
                        initial_path = a
                        break
            temp = {}
            temp['path'] = initial_path
            temp['length'] = self.get_walk_length(initial_path)
            temp['count'] = len(initial_path)
            update_path_list.append(temp)
        self.update_found.append(update_path_list)            
        # return path_comb_temp

    def assign_edges(self, path_comb):
        # [{}, {}, {}]
        edge_dict = {}
        unreached_edge = []    # 未经过的边
        for item in path_comb:
            path = item['path']
            for i in range(len(path)-1):
                if str([path[i], path[i+1]]) in edge_dict:
                    edge_dict[str([path[i], path[i+1]])] += 1
                elif str([path[i+1], path[i]]) in edge_dict:
                    edge_dict[str([path[i+1], path[i]])] += 1
                else:
                    edge_dict[str([path[i], path[i+1]])] = 1
        for item in self.__edges:
            if str([item[0], item[1]]) in edge_dict or str([item[1], item[0]]) in edge_dict:
                continue
            else:
                unreached_edge.append(item)
        
        assign_edges_list = []              #  [  [{}, {}, {}],   [],  [] ]
        for item in path_comb:
            assign_edges_list.append([])
        for edge in unreached_edge:
            distance, edge_flag, id_flag = sys.maxsize, None, None
            assign_id = sys.maxsize
            for i, value in enumerate(path_comb):
                dist_temp, edge_temp, id_temp = self.cal_edge_path_distance(value.get('path'), edge)
                if dist_temp < distance:
                    distance, edge_flag, id_flag = dist_temp, edge_temp, id_temp
                    assign_id = i
            assign_edges_list[assign_id].append({'edge':edge, 'dis':distance, 'edge_id':edge_flag, 'id':id_flag})
        for i in range(len(assign_edges_list)):
            assign_edges_list[i] = sorted(assign_edges_list[i], key=itemgetter('dis'))
        return assign_edges_list
        
    def cal_edge_path_distance(self, path_list, edge_temp):
        if edge_temp[0] in path_list:
            return 0, edge_temp[0], edge_temp[0]
        elif edge_temp[1] in path_list:
            return 0, edge_temp[1], edge_temp[1]
        else:
            result = sys.maxsize
            result_id = 0
            result_edge = None
            for idx in path_list:
                path_temp1 = self.Graph.graph.get_shortest_paths(idx, to=edge_temp[0], weights=self.Graph.graph.es["weight"], output="vpath",)[0]
                path_temp2 = self.Graph.graph.get_shortest_paths(idx, to=edge_temp[1], weights=self.Graph.graph.es["weight"], output="vpath",)[0]
                dist1 = self.Graph.get_length_graph(path_temp1)
                dist2 = self.Graph.get_length_graph(path_temp2)
                if min(dist1, dist2) < result:
                    result_id = idx
                    if dist1 < dist2:
                        result = dist1
                        result_edge = edge_temp[0]
                    else:    
                        result = dist2
                        result_edge = edge_temp[1]
            return result, result_edge, result_id
                

    def find_combinations(self):
        item_list = []
        for item in self.start_ids:
            item_list.append(item)
            self.aspect_comb(item, self.paths[item], self.start_ids[item], (), 0)         #aspect_found:  {item1: [ [{}, {}], [{}, {}]], }
        self.global_comb(self.aspect_found, item_list, (), 0)
        # result = [ [{}, {}, {}],  [{}, {}, {}],   [{}, {}, {}]      ]
        return True
    
    def global_comb(self, A, item_list, out=(), i=0):        
        if i >= len(A):
            self.found.append(list(out))
            return        
        for j in range(len(A[item_list[i]])):
            self.global_comb(A, item_list, out + (A[item_list[i]][j][0], ), i + 1)
        
    def aspect_comb(self, item, A, k, out=(), i=0):
        # k <= len(A) - i
        if len(A) == 0 or k > len(A):
            return
        if k == 0:
            self.aspect_found[item].append(list(out))
            return
        for j in range(i, len(A)):
            # add current element `A[j]` to the solution and recur for next index
            # `j+1` with one less element `k-1`
            self.aspect_comb(item, A, k - 1, out + (A[j], ), j + 1)

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

