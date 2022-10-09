from igraph import *
import random
import numpy as np

class WeightGraph():
    def __init__(self, env_vertices, env_edges):
        self.graph = None
        self.simpleGraph = None
        self.__r_edges = []
        self.__is1degree = False
        self.degree1 = []     # 度为1的顶点
        self.degree2 = []
        self.degree34 = []
        self.node = 0
        self.env_vertices = env_vertices
        self.env_edges = env_edges
        self.generate_graph()
        self.simple_the_graph()

    def generate_graph(self):
        self.graph = Graph()
        temp = len(self.env_vertices)
        number_of_vertex = 0
        for i in range(temp):
            number_of_vertex += np.shape(self.env_vertices[i])[0]
        number_of_edges = 0
        temp = len(self.env_edges)
        for i in range(temp):
            number_of_edges += np.shape(self.env_edges[i])[0]
            
        self.graph.add_vertices(number_of_vertex)
        self.node = number_of_vertex

        # label all the vertices
        aspect = 0
        acc = 0
        for i in range(number_of_vertex):
            self.graph.vs[i]["id"] = i
            self.graph.vs[i]['neighbor'] = []
            number = np.shape(self.env_vertices[aspect])[0]
            if i >= acc + number: 
                acc += number
                aspect += 1
            self.graph.vs[i]["position"] = self.env_vertices[aspect][i - acc]
            
        edges = []
        weights = []
        for item in self.env_edges:
            for tu in item:    # 4
                position1, position2 = tu[0], tu[1]   # （3, ）np.ndarray 
                flag1, flag2 = None, None
                for i in range(number_of_vertex):
                    if np.array_equal(self.graph.vs[i]["position"], position1):
                        flag1 = self.graph.vs[i]['id']
                    elif np.array_equal(self.graph.vs[i]["position"], position2):
                        flag2 = self.graph.vs[i]['id']
                    if flag1 is not None and flag2 is not None:
                        edge = [flag1, flag2]
                        self.graph.vs[flag1]['neighbor'].append(flag2)
                        self.graph.vs[flag2]['neighbor'].append(flag1)
                        edges.append(edge)
                        # weight = self.cal_weight(edge)
                        # weights.append(weight)
                        break
        self.graph.add_edges(edges)     
        self.graph.es['weight'] = [1] * len(edges)
        self.graph.es['label'] = [1] * len(edges)
        self.graph.es["curved"] = False
        degrees = [0] * number_of_vertex
        for i in range(number_of_vertex):
            degrees[i] = self.graph.degree(i)
            if degrees[i] == 1:
                self.__is1degree = True
                graf = self.get_edges()
                adj = self.get_adj(i,graf)
                self.degree1.append(i)
            elif degrees[i] == 2:
                self.degree2.append(i)
            else:
                self.degree34.append(i)


    def simple_the_graph(self):
        self.simpleGraph = Graph()
        
        """
        num = len(self.degree1) + len(self.degree34)
        self.simpleGraph.add_vertices(num)
        for i in range(num):
            if i < len(self.degree1):
                self.simpleGraph.vs[i]['id'] = self.degree1[i]
            else:
                self.simpleGraph.vs[i]['id'] = self.degree34[i - len(self.degree1)]
        """
        num = len(self.degree1) + len(self.degree34) + len(self.degree2)
        self.simpleGraph.add_vertices(num)
        for i in range(num):
            self.simpleGraph.vs[i]["id"] = self.graph.vs[i]["id"]
            self.simpleGraph.vs[i]["position"] = self.graph.vs[i]["position"]
        
        edges_temp = []
        for edge in self.graph.get_edgelist():
            if edge[0] not in self.degree2 and edge[1] not in self.degree2:
                edges_temp.append(list(edge))
        weights = [1] * len(edges_temp)
        id_pass = []
        for idx in self.degree2:
            if idx not in id_pass:
                edge_last, id_list, length = self.find_path(idx)
                edges_temp.append(edge_last)
                weights.extend([length])
                id_pass.extend(id_list)
        self.simpleGraph.add_edges(edges_temp)        
        self.simpleGraph.es['weight'] = weights
        self.simpleGraph.es['label'] = weights
        self.simpleGraph.es["curved"] = False
        
        
                
    def find_path(self, idx):
        temp = []
        length = 0
        current_flag = idx
        left_flag, right_flag = self.graph.vs[idx]['neighbor'][0], self.graph.vs[idx]['neighbor'][1]
        while left_flag in self.degree2:
            temp.append(left_flag)
            a, b = self.graph.vs[left_flag]['neighbor'][0], self.graph.vs[left_flag]['neighbor'][1]
            if a == current_flag:
                left_flag = b
            else:
                left_flag = a
            current_flag = left_flag
            length += 1
        current_flag = idx
        while right_flag in self.degree2:
            temp.append(right_flag)
            a, b = self.graph.vs[right_flag]['neighbor'][0], self.graph.vs[right_flag]['neighbor'][1]
            if a == current_flag:
                right_flag = b
            else:
                right_flag = a
            current_flag = right_flag
            length += 1
        return [left_flag, right_flag], temp, length
        
    
    def get_adj(self, node, graf):    #邻接节点
        new_edges = []
        for edge in graf:
            if node in edge:
                temp = [edge[0], edge[1]]
                temp.remove(node)
                new_edges.append(temp[0])
        return new_edges

    def is_in(self, value, edges):
        if value in edges:
            return True
        temp_val = [value[1], value[0]]
        if temp_val in edges:
            return True
        return False

    def generate_position_of_path(self, walks):
        position_paths = []
        for item in walks:
            # item['path'] = []
            edge_list = []
            temp = item['path']
            for i in range(len(temp)-1):
                id1, id2 = temp[i], temp[i+1]            
                pos1, pos2 = None, None
                for j in range(self.node):
                    if self.graph.vs[j]['id'] == id1:
                        pos1 = self.graph.vs[j]['position']
                    elif self.graph.vs[j]['id'] == id2:
                        pos2 = self.graph.vs[j]['position']
                    if pos1 is not None and pos2 is not None:
                        edge = [pos1, pos2]
                        edge_list.append(edge)
                        break
            position_paths.append(edge_list)        
        return position_paths

    def trans_paths(self, paths):
        result = []
        for item in paths:
            cache = item['path'].copy()
            r = self.trans_path(cache)
            result.append({'path':r})
        return result

    def trans_path(self, path):
        # trans path in simple Graph to path in original graph
        result = [path[0]]
        n = len(path)
        flag = 0
        while flag < n - 1:
            a, b = path[flag], path[flag+1]
            if b in self.graph.vs[a]['neighbor']:
                result.append(b)
            else:
                path_temp = self.get_shortest_path(self, a, b)
                temp = path_temp[1:]
                result.extend(temp)
            flag += 1
        return result
        


    def get_edges(self):
        return self.graph.get_edgelist()

    def get_is1Degree(self):
        return self.__is1degree

    def get_degree1(self):
        return self.degree1

    def get_graph(self):
        return self.graph

    def get_weights(self):
        return self.graph.es['weight']


    def get_shortest_path(self, start_node, destination_node):
        return self.graph.get_shortest_paths(
            start_node,
            to=destination_node,
            weights=self.graph.es["weight"],
            output="vpath",
        )

    def get_shortest_path_length(self, path):
        if len(path[0]) > 0:
            # Add up the weights across all edges on the shortest path
            distance = 0
            n = len(path[0])
            for i in range(0, n):
                if i + 1 != n:
                    mytuple = (path[0][i], path[0][i + 1])
                    distance += self.get_weight_by_index(self.get_edges().index(mytuple))
            return distance
        else:
            print("shortest path is empty")
            return 0
            
    def get_length_simple_graph(self, path):
        if len(path) > 0:
            # Add up the weights across all edges on the shortest path
            distance = 0
            n = len(path)
            for i in range(n-1):
                mytuple = (path[i], path[i + 1])
                if mytuple in self.simpleGraph.get_edgelist():
                    distance += self.get_weight_by_index(self.simpleGraph.get_edgelist().index(mytuple))
                else:
                    mytuple = (path[i+1], path[i])
                    if mytuple in self.simpleGraph.get_edgelist():
                        distance += self.get_weight_by_index(self.simpleGraph.get_edgelist().index(mytuple))
                    else:
                        print("--------------------ERROR")
                        break
            return distance
        else:
            print("shortest path is empty")
            return 0

    def get_weight_by_index(self, index):
        return (self.graph.es["weight"])[index]