from operator import itemgetter
import time
import sys


# I didn't want to bother with global variables therefore,
# I created a class to encapsulate my algorithm.
class HeuristicAlgorithm():

    def __init__(self):
        self.__my_graph = None
        self.__edges = None
        self.__sorted_edges = []
        self.__closed_walks = []
        self.__second_closed_walks = []
        self.__walks_lengths = []
        self.__initial_vertex = 0
        self.__k = 0
        self.__n = 0

    def my_algorithm(self, k):
        self.__k = k
        start_time = time.perf_counter()
        self.sort_edges_descending()
        print("sorted edges:")
        print(self.__sorted_edges)
        self.create_closed_walk(k)
        total_time = (time.perf_counter() - start_time)
        print("cycles:")
        print(self.__closed_walks)
        print(total_time)
        return [self.__closed_walks, total_time]
        # self.simple_algo()
        # print("cycles2:")
        # print(self.__second_closed_walks)



    def sort_edges_descending(self):
        weights = self.__my_graph.get_weights()
        edges = self.__my_graph.get_edges()
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

    def create_closed_walk(self, k):
        # for each edge
        for e in self.__sorted_edges:

            # e = {vi, vj}
            path3 = [e['start_node'], e['end_node']]
            walk = []
            # if e is not already covered by an existing tour.
            if not self.check_added(path3):
                # SP(v1, vi)
                path1 = self.__my_graph.get_shortest_path(self.__my_graph.get_initial_vertex(), path3[0])[0]
                # SP(vj, v1)
                path2 = self.__my_graph.get_shortest_path(path3[1], self.__my_graph.get_initial_vertex())[0]

                # try to create closed walk
                if self.try_to_merge(path1, path2, walk):
                    self.add_edge_to_walk(walk, path3)
                # try to create closed walk
                elif self.try_to_merge(path1, path3, walk):
                    self.add_edge_to_walk(walk, path2)
                # try to create closed walk
                elif self.try_to_merge(path2, path3, walk):
                    self.add_edge_to_walk(walk, path1)
                else:
                    walk.extend(self.get_maximum(path1, path2, path3))
                if len(walk) > 1:
                    if walk[0] != walk[-1] and self.is_in_edge_list([walk[0], walk[-1]]):
                        walk.append(walk[0])
                    self.__closed_walks.append(
                        {'cycle': walk, 'length': self.get_walk_length(walk), 'count': len(walk)})

        self.__closed_walks = sorted(self.__closed_walks, key=itemgetter('length'), reverse=True)
        print(self.__closed_walks)
        if len(self.__closed_walks) < k:
            self.add_dummy_tours(k - len(self.__closed_walks))
            self.__closed_walks = sorted(self.__closed_walks, key=itemgetter('length'), reverse=True)
        elif len(self.__closed_walks) > k:
            self.merge_tours(k)
            self.__closed_walks = sorted(self.__closed_walks, key=itemgetter('length'), reverse=True)
        print("len")
        print(len(self.__closed_walks))

    def get_maximum(self, a, b, c):

        if (len(a) >= len(b)) and (len(a) >= len(c)):
            largest = a

        elif (len(b) >= len(a)) and (len(b) >= len(c)):
            largest = b
        else:
            largest = c

        return largest

    def check_added(self, edge):
        for e in self.__closed_walks:
            walk = e['cycle']
            if self.sub_list_exists(walk, edge):
                return True
            reverse_edge = [edge[1], edge[0]]
            if self.sub_list_exists(walk, reverse_edge):
                return True
        return False

    def add_dummy_tours(self, missing_number):
        listLen = len(self.__sorted_edges)
        for i in range(listLen):
            e = self.__sorted_edges[(listLen - i) - 1]
            walk = [e['end_node'], e['start_node'], e['end_node']]
            if self.__initial_vertex in walk:
                for j in range(missing_number):
                    self.__closed_walks.append(
                        {'cycle': walk, 'length': self.get_walk_length(walk), 'count': len(walk)})
                break

    def merge_tours(self, k):
        listLen = len(self.__closed_walks)
        n = listLen - k
        is_ok = False
        for i in range(n):
            if i == 0:
                if self.merge_round2():
                    print("round2")
                    is_ok = True
                    listLen = len(self.__closed_walks)
                    if listLen == k:
                        return True
                if not is_ok and self.merge_round1():
                    print("round1")
                    is_ok = True
                    listLen = len(self.__closed_walks)
                    if listLen == k:
                        return True
            if i > 0 and is_ok:
                is_ok = False
                if self.merge_round2():
                    print("round2")
                    is_ok = True
                    listLen = len(self.__closed_walks)
                    if listLen == k:
                        return True
                if not is_ok and self.merge_round1():
                    print("round1")
                    is_ok = True
                    listLen = len(self.__closed_walks)
                    if listLen == k:
                        return True

    def merge_round1(self):
        listLen = len(self.__closed_walks)
        """
        for i in range(listLen):
            sm_el = self.__closed_walks[listLen - i - 1]
            walk_path = sm_el['cycle']
            for j in range(i, listLen - 1):
                next1 = self.__closed_walks[(listLen - j - 1) - 1]
                next_path = next1['cycle']
                if walk_path[-1] == next_path[0]:
                    next_path.pop()
                    next_path.extend(walk_path)
                    self.__closed_walks[(listLen - j - 1) - 1] = {'cycle': next_path,
                                                                  'length': self.get_walk_length(next_path),
                                                                  'count': len(next_path)}
                    del self.__closed_walks[listLen - i - 1]
                    self.__closed_walks = sorted(self.__closed_walks, key=itemgetter('length'), reverse=True)
                    return True
        """
        breakk = False
        for i in range(listLen):
            sm_el = self.__closed_walks[listLen - i - 1]
            walk_path = sm_el['cycle']
            for j in range(i, listLen - 1):
                if j < 2:
                    next1 = self.__closed_walks[(listLen - j - 1) - 1]
                    next_path = next1['cycle']
                    if walk_path[0] == next_path[-1]:
                        next_path.pop()
                        next_path.extend(walk_path)
                        self.__closed_walks[(listLen - j - 1) - 1] = {'cycle': next_path,
                                                                      'length': self.get_walk_length(next_path),
                                                                      'count': len(next_path)}
                        del self.__closed_walks[listLen - i - 1]
                        self.__closed_walks = sorted(self.__closed_walks, key=itemgetter('length'), reverse=True)
                        return True
                    if self.is_in_edge_list([walk_path[0], next_path[-1]]):
                        walk_path.append(next_path[-1])
                        next_path.extend(walk_path)
                        self.__closed_walks[(listLen - j - 1) - 1] = {'cycle': next_path,
                                                                      'length': self.get_walk_length(next_path),
                                                                      'count': len(next_path)}
                        del self.__closed_walks[listLen - i - 1]
                        self.__closed_walks = sorted(self.__closed_walks, key=itemgetter('length'), reverse=True)
                        return True
                elif i != (listLen - 1) and (listLen - (j - 1) - 1) - 1 != listLen - (i + 1) - 1:
                    next12 = self.__closed_walks[(listLen - (j - 1) - 1) - 1]
                    next_path2 = next12['cycle']
                    sm_el1 = self.__closed_walks[listLen - (i + 1) - 1]
                    walk_path1 = sm_el1['cycle']
                    if walk_path1[0] == next_path2[-1]:
                        next_path2.pop()
                        next_path2.extend(walk_path1)
                        self.__closed_walks[(listLen - (j - 1) - 1) - 1] = {'cycle': next_path2,
                                                                            'length': self.get_walk_length(next_path2),
                                                                            'count': len(next_path2)}
                        del self.__closed_walks[listLen - (i + 1) - 1]
                        self.__closed_walks = sorted(self.__closed_walks, key=itemgetter('length'), reverse=True)
                        return True
                    if self.is_in_edge_list([walk_path1[0], next_path2[-1]]):
                        walk_path1.append(next_path2[-1])
                        next_path2.extend(walk_path1)
                        self.__closed_walks[(listLen - (j - 1) - 1) - 1] = {'cycle': next_path2,
                                                                            'length': self.get_walk_length(next_path2),
                                                                            'count': len(next_path2)}
                        del self.__closed_walks[listLen - (i + 1) - 1]
                        self.__closed_walks = sorted(self.__closed_walks, key=itemgetter('length'), reverse=True)
                        return True
                if j > 1:
                    next1 = self.__closed_walks[(listLen - j - 1) - 1]
                    next_path = next1['cycle']
                    if walk_path[0] == next_path[-1]:
                        next_path.pop()
                        next_path.extend(walk_path)
                        self.__closed_walks[(listLen - j - 1) - 1] = {'cycle': next_path,
                                                                      'length': self.get_walk_length(next_path),
                                                                      'count': len(next_path)}
                        del self.__closed_walks[listLen - i - 1]
                        self.__closed_walks = sorted(self.__closed_walks, key=itemgetter('length'), reverse=True)
                        return True
                    if self.is_in_edge_list([walk_path[0], next_path[-1]]):
                        walk_path.append(next_path[-1])
                        next_path.extend(walk_path)
                        self.__closed_walks[(listLen - j - 1) - 1] = {'cycle': next_path,
                                                                      'length': self.get_walk_length(next_path),
                                                                      'count': len(next_path)}
                        del self.__closed_walks[listLen - i - 1]
                        self.__closed_walks = sorted(self.__closed_walks, key=itemgetter('length'), reverse=True)
                        return True

        return False

    def merge_round2(self):
        listLen = len(self.__closed_walks)
        self.__closed_walks = sorted(self.__closed_walks, key=itemgetter('count'))

        for i in range(listLen):
            sm_el = self.__closed_walks[i]
            sm_walk = sm_el['cycle']
            for j in range(i + 1, listLen):
                big_el = self.__closed_walks[j]
                big_walk = big_el['cycle']
                n = len(sm_walk)
                big_ok = True
                for k in range(0, n):
                    if k + 1 != n:
                        edge = [sm_walk[k], sm_walk[k + 1]]
                        is_ok = False
                        if self.sub_list_exists(big_walk, edge):
                            is_ok = True
                        edge.reverse()
                        if self.sub_list_exists(big_walk, edge):
                            is_ok = True
                        if not is_ok:
                            big_ok = False
                if big_ok:
                    del self.__closed_walks[i]
                    self.__closed_walks = sorted(self.__closed_walks, key=itemgetter('length'), reverse=True)
                    return True
        self.__closed_walks = sorted(self.__closed_walks, key=itemgetter('length'), reverse=True)
        return False

    def sub_list_exists(self, list1, list2):
        if len(list2) < 2:
            return False
        return ''.join(map(str, list2)) in ''.join(map(str, list1))

    def try_to_merge(self, path1, path2, walk):

        if len(path1) > 1 and len(path2) > 1 and \
                (not self.sub_list_exists(path1, path2)):
            if path1[-1] == path2[0]:
                if len(walk) >= len(path1):
                    if not self.sub_list_exists(walk, path1):
                        walk.extend(path1)
                else:
                    walk.extend(path1)

                if len(walk) >= len(path2[1:]):
                    if not self.sub_list_exists(walk, path2[1:]):
                        walk.extend(path2[1:])
                else:
                    walk.extend(path2[1:])

                return True

            elif self.is_in_edge_list([path1[0], path2[-1]]):
                if len(walk) >= len(path2):
                    if not self.sub_list_exists(walk, path2):
                        walk.extend(path2)
                else:
                    walk.extend(path2)

                if len(walk) >= len(path1):
                    if not self.sub_list_exists(walk, path1):
                        walk.extend(path1)
                else:
                    walk.extend(path1)

                return True

            elif self.is_in_edge_list([path1[-1], path2[0]]):
                if len(walk) >= len(path1):
                    if not self.sub_list_exists(walk, path1):
                        walk.extend(path1)
                else:
                    walk.extend(path1)

                if len(walk) >= len(path2):
                    if not self.sub_list_exists(walk, path2):
                        walk.extend(path2)
                else:
                    walk.extend(path2)

                return True
            elif path1[0] == path2[-1]:
                if len(walk) >= len(path2):
                    if not self.sub_list_exists(walk, path2):
                        walk.extend(path2)
                else:
                    walk.extend(path2)

                if len(walk) >= len(path1[1:]):
                    if not self.sub_list_exists(walk, path1[1:]):
                        walk.extend(path1[1:])
                else:
                    walk.extend(path1[1:])
                return True

            else:
                return False
        else:
            return False

    def add_edge_to_walk(self, walk, path3):
        # walk eğer path'ü içermiyorsa path3'u ekle
        if len(walk) > 1 and len(path3) > 1 and \
                (not self.sub_list_exists(walk, path3)):
            # eğer path1'in son node'u ile path3'un ilk node'u eşitse
            if walk[-1] == path3[0]:
                if len(walk) >= len(path3[1:]):
                    if not self.sub_list_exists(walk, path3[1:]):
                        walk.extend(path3[1:])
                else:
                    walk.extend(path3[1:])

            elif self.is_in_edge_list([walk[-1], path3[0]]):
                if len(walk) >= len(path3):
                    if not self.sub_list_exists(walk, path3):
                        walk.extend(path3)
                else:
                    walk.extend(path3)

            elif self.is_in_edge_list([walk[0], path3[-1]]):
                if len(path3) >= len(walk):
                    if not self.sub_list_exists(path3, walk):
                        path3.extend(walk)
                else:
                    path3.extend(walk)

            elif walk[0] == path3[-1]:

                if len(path3) >= len(walk[1:]):
                    if not self.sub_list_exists(path3, walk[1:]):
                        path3.extend(walk[1:])
                else:
                    path3.extend(walk[1:])



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

