from igraph import *
import random
import numpy as np

class WeightGraph():
    def __init__(self, env_vertices, env_edges):
        self.graph = None
        self.__r_edges = []
        self.__is1degree = False
        self.__degree1 = []     # 度为1的顶点
        self.node = 0
        self.env_vertices = env_vertices
        self.env_edges = env_edges
        self.generate_graph()

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
                        edges.append(edge)
                        # weight = self.cal_weight(edge)
                        # weights.append(weight)
                        break
        self.graph.add_edges(edges)        
        self.graph.es['weight'] = 1
        self.graph.es['label'] = 1
        self.graph.es["curved"] = False
        degrees = [0] * number_of_vertex
        for i in range(number_of_vertex):
            degrees[i] = self.graph.degree(i)
            if degrees[i] == 1:
                self.__is1degree = True
                graf = self.get_edges()
                adj = self.get_adj(i,graf)
                lst = (i, adj[0])
                lst2 = [i, adj[0]]
                if lst in graf:
                    self.__degree1.append(lst2)
                else:
                    lst2.reverse()
                    self.__degree1.append(lst2)

        
        
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

    def get_edges(self):
        return self.graph.get_edgelist()

    def get_edges2(self):
        return self.__r_edges

    def get_is1Degree(self):
        return self.__is1degree

    def get_degree1(self):
        return self.__degree1

    def get_graph(self):
        return self.graph

    def get_weights(self):
        return self.graph.es['weight']

    def print_info(self):
        print("----------Information-----------")
        print("Number of vertices in the graph:", self.graph.vcount())
        print("Number of edges in the graph", self.graph.ecount())
        print("Is the graph directed:", self.graph.is_directed())
        print("Maximum degree in the graph:", self.graph.maxdegree())
        print("Adjacency matrix:\n", self.graph.get_adjacency())
        print("weights:", self.graph.es['weight'])

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

    def get_weight_by_index(self, index):
        return (self.graph.es["weight"])[index]