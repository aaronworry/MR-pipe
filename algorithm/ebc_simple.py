"""
Simplified EBC algorithm: single dict input `graph` with node indices, edges, and
`start_ids` (the same mapping `EBCAlgorithm.get_start_ids` produces:
{start_node_id: robot_count}).
"""
from operator import itemgetter
import sys
import numpy as np
from igraph import Graph


def _parse_edge(e):
    if isinstance(e, dict):
        u = int(e["u"])
        v = int(e["v"])
        w = float(e.get("w", e.get("weight", 1.0)))
        return u, v, w
    if len(e) == 2:
        return int(e[0]), int(e[1]), 1.0
    return int(e[0]), int(e[1]), float(e[2])


def _infer_n(edges_parsed, graph_dict):
    if "n" in graph_dict:
        return int(graph_dict["n"])
    if "nodes" in graph_dict:
        nodes = graph_dict["nodes"]
        return max(int(x) for x in nodes) + 1 if nodes else 0
    if not edges_parsed:
        return 0
    m = 0
    for u, v, _ in edges_parsed:
        m = max(m, u + 1, v + 1)
    return m


class DictWeightGraph:
    """Minimal backend matching what EBCAlgorithm expects from WeightGraph."""

    def __init__(self, n, edges_parsed):
        self.graph = Graph()
        self.degree1 = []
        self.degree2 = []
        self.degree34 = []
        self.degree134 = []
        self.node = n
        self.graph.add_vertices(n)
        edgelist = []
        weights = []
        for u, v, w in edges_parsed:
            edgelist.append((u, v))
            weights.append(w)
        for i in range(n):
            self.graph.vs[i]["id"] = i
            self.graph.vs[i]["neighbor"] = []
        for (u, v), w in zip(edgelist, weights):
            self.graph.vs[u]["neighbor"].append(v)
            self.graph.vs[v]["neighbor"].append(u)
        self.graph.add_edges(edgelist)
        self.graph.es["weight"] = weights
        self.graph.es["label"] = [1] * len(weights)
        self.graph.es["curved"] = False
        degrees = [self.graph.degree(i) for i in range(n)]
        for i in range(n):
            if degrees[i] == 1:
                self.degree1.append(i)
                self.degree134.append(i)
            elif degrees[i] == 2:
                self.degree2.append(i)
            else:
                self.degree34.append(i)
                self.degree134.append(i)

    def get_edges(self):
        return self.graph.get_edgelist()

    def get_adj(self, node, graf):
        new_edges = []
        for edge in graf:
            if node in edge:
                temp = [edge[0], edge[1]]
                temp.remove(node)
                new_edges.append(temp[0])
        return new_edges

    def get_length_graph(self, path):
        if len(path) > 0:
            distance = 0
            el = self.graph.get_edgelist()
            for i in range(len(path) - 1):
                mytuple = (path[i], path[i + 1])
                if mytuple in el:
                    distance += self.get_weight_by_index(el.index(mytuple))
                else:
                    mytuple = (path[i + 1], path[i])
                    if mytuple in el:
                        distance += self.get_weight_by_index(el.index(mytuple))
                    else:
                        break
            return distance
        return 0

    def get_weight_by_index(self, index):
        return self.graph.es["weight"][index]


class EBCSimpleAlgorithm:
    def __init__(self, graph):
        """
        graph: dict with keys
          - "edges": list of [u, v], [u, v, weight], or {"u","v","w"?}
          - "start_ids": {node_id: count} same as EBCAlgorithm.get_start_ids
          - optional "n" or "nodes" for vertex count (else inferred from edges)
        """
        if "edges" not in graph or "start_ids" not in graph:
            raise KeyError('graph must contain "edges" and "start_ids"')
        edges_parsed = [_parse_edge(e) for e in graph["edges"]]
        n = _infer_n(edges_parsed, graph)
        self.Graph = DictWeightGraph(n, edges_parsed)
        self.start_ids = {int(k): int(v) for k, v in graph["start_ids"].items()}
        self.__edges = None
        self.__sorted_edges = []
        self.found = []
        self.update_found = []
        self.aspect_found = {}
        self.paths = {}
        self.result = []
        for nid in self.start_ids:
            self.paths[nid] = []
            self.aspect_found[nid] = []
        self.k = sum(self.start_ids.values())
        self.graph = [list(elem) for elem in self.Graph.get_edges()]

    def my_algorithm(self):
        self.sort_edges_descending()
        self.create_paths()
        unvisited_edge_num, sum_visited_edge = self.checkResult(self.result)
        qq = len(self.graph) - unvisited_edge_num
        repe = (sum_visited_edge - qq) / qq if qq else 0.0
        return unvisited_edge_num, repe, self.result

    def checkResult(self, paths):
        temp_mat = np.zeros((self.Graph.node, self.Graph.node))
        for item in paths:
            path = item["path"]
            for i in range(len(path) - 1):
                if path[i] != path[i + 1]:
                    temp_mat[path[i]][path[i + 1]] += 1
                    temp_mat[path[i + 1]][path[i]] += 1
        sum_visited_edge = np.sum(temp_mat) / 2
        return 0, sum_visited_edge

    def sort_edges_descending(self):
        weights = list(self.Graph.graph.es["weight"])
        edges = list(self.Graph.graph.get_edgelist())
        self.__edges = edges
        n = len(weights)
        for i in range(n):
            for j in range(0, n - i - 1):
                if weights[j] < weights[j + 1]:
                    edges[j], edges[j + 1] = edges[j + 1], edges[j]
                    weights[j], weights[j + 1] = weights[j + 1], weights[j]
        self.create_edge_dict(edges, weights)

    def create_edge_dict(self, edges, weights):
        for i in range(len(weights)):
            edge = edges[i]
            self.__sorted_edges.append(
                {"start_node": edge[0], "end_node": edge[1], "length": weights[i]}
            )

    def get_edge_length(self, edge):
        for e in self.__sorted_edges:
            if (e["start_node"] == edge[0] and e["end_node"] == edge[1]) or (
                e["start_node"] == edge[1] and e["end_node"] == edge[0]
            ):
                return e["length"]
        return 0

    def is_in_edge_list(self, edge):
        for e in self.__sorted_edges:
            if (e["start_node"] == edge[0] and e["end_node"] == edge[1]) or (
                e["start_node"] == edge[1] and e["end_node"] == edge[0]
            ):
                return True
        return False

    def create_paths(self):
        for e in self.__sorted_edges:
            path_temp = [e["start_node"], e["end_node"]]
            if not self.check_added_all(path_temp, self.paths):
                temp_item, temp_walk = 0, {"path": None, "length": sys.maxsize, "count": 0}
                for item in self.start_ids:
                    walk = []
                    path1 = self.Graph.graph.get_shortest_paths(
                        item,
                        to=path_temp[0],
                        weights=self.Graph.graph.es["weight"],
                        output="vpath",
                    )[0]
                    path2, distance = None, sys.maxsize
                    for goal in self.Graph.degree1:
                        path = self.Graph.graph.get_shortest_paths(
                            path_temp[1],
                            to=goal,
                            weights=self.Graph.graph.es["weight"],
                            output="vpath",
                        )[0]
                        temp = self.Graph.get_length_graph(path)
                        if temp <= distance:
                            path2 = path
                            distance = temp
                    self.try_to_merge(path1, path2, walk)
                    path_temp1 = {
                        "path": walk,
                        "length": self.get_walk_length(walk),
                        "count": len(walk),
                    }

                    walk = []
                    path1 = self.Graph.graph.get_shortest_paths(
                        item,
                        to=path_temp[1],
                        weights=self.Graph.graph.es["weight"],
                        output="vpath",
                    )[0]
                    path2, distance = None, sys.maxsize
                    for goal in self.Graph.degree1:
                        path = self.Graph.graph.get_shortest_paths(
                            path_temp[0],
                            to=goal,
                            weights=self.Graph.graph.es["weight"],
                            output="vpath",
                        )[0]
                        temp = self.Graph.get_length_graph(path)
                        if temp <= distance:
                            path2 = path
                            distance = temp
                    self.try_to_merge(path1, path2, walk)
                    path_temp2 = {
                        "path": walk,
                        "length": self.get_walk_length(walk),
                        "count": len(walk),
                    }

                    if path_temp1["length"] <= path_temp2["length"]:
                        if path_temp1["length"] < temp_walk["length"]:
                            temp_item, temp_walk = item, path_temp1
                    else:
                        if path_temp2["length"] < temp_walk["length"]:
                            temp_item, temp_walk = item, path_temp2
                self.paths[temp_item].append(temp_walk)
        for item in self.paths:
            self.paths[item] = sorted(self.paths[item], key=itemgetter("length"), reverse=True)
        for i in self.start_ids:
            if self.start_ids[i] > len(self.paths[i]):
                self.add_dummy_tours(i, self.start_ids[i] - len(self.paths[i]))
        result = self.merge_walks()
        for item in result:
            if item["count"] == 1:
                item["count"] = 3
                item["length"] = 2
                temp_path_node = item["path"][0]
                new_node = self.Graph.get_adj(temp_path_node, self.Graph.get_edges())[0]
                item["path"].append(new_node)
                item["path"].append(temp_path_node)
        self.result = result

    def check_added(self, edge, paths):
        for e in paths:
            walk = e["path"]
            if self.sub_list_exists(walk, edge):
                return True
            reverse_edge = [edge[1], edge[0]]
            if self.sub_list_exists(walk, reverse_edge):
                return True
        return False

    def check_added_all(self, edge, paths_dict):
        for item in self.start_ids:
            for e in paths_dict[item]:
                walk = e["path"]
                if self.sub_list_exists(walk, edge):
                    return True
                reverse_edge = [edge[1], edge[0]]
                if self.sub_list_exists(walk, reverse_edge):
                    return True
        return False

    def add_dummy_tours(self, idx, missing_number):
        for _ in range(missing_number):
            self.paths[idx].append({"path": [idx], "length": 0, "count": 1})

    def merge_walks(self):
        self.find_combinations()
        for path_comb in self.found:
            edge_assigned_list = self.assign_edges(path_comb)
            self.update_paths(path_comb, edge_assigned_list)
        r_id, max_l = None, sys.maxsize
        for index, path_list in enumerate(self.update_found):
            max_length = 0
            for path in path_list:
                if max_length <= path["length"]:
                    max_length = path["length"]
            if max_length <= max_l:
                max_l = max_length
                r_id = index
        return self.update_found[r_id]

    def update_paths(self, path_comb, edge_assigned_list):
        update_path_list = []
        for i, value in enumerate(path_comb):
            initial_path = value["path"].copy()
            for edge in edge_assigned_list[i]:
                dist_temp, edge_temp, id_temp = self.cal_edge_path_distance(
                    initial_path, edge["edge"]
                )
                insert_path = self.Graph.graph.get_shortest_paths(
                    id_temp,
                    to=edge_temp,
                    weights=self.Graph.graph.es["weight"],
                    output="vpath",
                )[0]
                path_temp2 = insert_path.copy()
                path_temp2.reverse()
                path_one = (
                    edge["edge"][0] if edge["edge"][1] == edge_temp else edge["edge"][1]
                )
                insert_path.extend([path_one])
                insert_path.extend(path_temp2)
                for index, idx in enumerate(initial_path):
                    if idx == id_temp:
                        a = initial_path[:index]
                        b = initial_path[index + 1 :]
                        a.extend(insert_path)
                        a.extend(b)
                        initial_path = a
                        break
            temp = {
                "path": initial_path,
                "length": self.get_walk_length(initial_path),
                "count": len(initial_path),
            }
            update_path_list.append(temp)
        self.update_found.append(update_path_list)

    def assign_edges(self, path_comb):
        edge_dict = {}
        unreached_edge = []
        temp_path_length = [0] * len(path_comb)
        for item in path_comb:
            path = item["path"]
            for i in range(len(path) - 1):
                key_f = str([path[i], path[i + 1]])
                key_r = str([path[i + 1], path[i]])
                if key_f in edge_dict:
                    edge_dict[key_f] += 1
                elif key_r in edge_dict:
                    edge_dict[key_r] += 1
                else:
                    edge_dict[key_f] = 1
        for item in self.__edges:
            if (
                str([item[0], item[1]]) in edge_dict
                or str([item[1], item[0]]) in edge_dict
            ):
                continue
            unreached_edge.append(item)

        assign_edges_list = [[] for _ in path_comb]
        for edge in unreached_edge:
            distance, edge_flag, id_flag = sys.maxsize, None, None
            assign_id = sys.maxsize
            for i, value in enumerate(path_comb):
                dist_temp, edge_temp, id_temp = self.cal_edge_path_distance(
                    value.get("path"), edge
                )
                if 2 + 2 * dist_temp + temp_path_length[i] < distance:
                    distance = 2 + 2 * dist_temp + temp_path_length[i]
                    edge_flag, id_flag = edge_temp, id_temp
                    assign_id = i
            assign_edges_list[assign_id].append(
                {"edge": edge, "dis": distance, "edge_id": edge_flag, "id": id_flag}
            )
            temp_path_length[assign_id] += 2 + 2 * dist_temp
        for i in range(len(assign_edges_list)):
            assign_edges_list[i] = sorted(assign_edges_list[i], key=itemgetter("dis"))
        return assign_edges_list

    def cal_edge_path_distance(self, path_list, edge_temp):
        if edge_temp[0] in path_list:
            return 0, edge_temp[0], edge_temp[0]
        if edge_temp[1] in path_list:
            return 0, edge_temp[1], edge_temp[1]
        result = sys.maxsize
        result_id = 0
        result_edge = None
        for idx in path_list:
            path_temp1 = self.Graph.graph.get_shortest_paths(
                idx,
                to=edge_temp[0],
                weights=self.Graph.graph.es["weight"],
                output="vpath",
            )[0]
            path_temp2 = self.Graph.graph.get_shortest_paths(
                idx,
                to=edge_temp[1],
                weights=self.Graph.graph.es["weight"],
                output="vpath",
            )[0]
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
            self.aspect_comb(item, self.paths[item], self.start_ids[item], (), 0)
        self.global_comb(self.aspect_found, item_list, (), 0)
        return True

    def global_comb(self, a, item_list, out=(), i=0):
        if i >= len(a):
            self.found.append(list(out))
            return
        for j in range(len(a[item_list[i]])):
            temp_co = out
            for item in a[item_list[i]][j]:
                temp_co = temp_co + (item,)
            self.global_comb(a, item_list, temp_co, i + 1)

    def aspect_comb(self, item, a, k, out=(), i=0):
        if len(a) == 0 or k > len(a):
            return
        if k == 0:
            self.aspect_found[item].append(list(out))
            return
        for j in range(i, len(a)):
            self.aspect_comb(item, a, k - 1, out + (a[j],), j + 1)

    def sub_list_exists(self, list1, list2):
        if len(list2) < 2:
            return False
        return "".join(map(str, list2)) in "".join(map(str, list1))

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
            distance = 0
            n = len(walk)
            for i in range(0, n - 1):
                edge = [walk[i], walk[i + 1]]
                distance += self.get_edge_length(edge)
            return distance
        print("walk is empty")
        return 0


def _build_demo_graph_100_complex():
    """
    100 nodes: 0..98 form a dense 9x11 grid (orthogonal + both diagonals + knight
    moves + deterministic long chords); node 99 is a leaf (degree 1) attached to a hub.
    """
    edges_set = set()

    def add_e(u, v):
        u, v = int(u), int(v)
        if u == v:
            return
        if u > v:
            u, v = v, u
        edges_set.add((u, v))

    rows, cols = 9, 11

    def ix(r, c):
        return r * cols + c

    for r in range(rows):
        for c in range(cols):
            u = ix(r, c)
            if c < cols - 1:
                add_e(u, u + 1)
            if r < rows - 1:
                add_e(u, u + cols)
    for r in range(rows - 1):
        for c in range(cols - 1):
            add_e(ix(r, c), ix(r + 1, c + 1))
    for r in range(rows - 1):
        for c in range(1, cols):
            add_e(ix(r, c), ix(r + 1, c - 1))
    knight = ((2, 1), (1, 2), (-1, 2), (-2, 1), (-2, -1), (-1, -2), (1, -2), (2, -1))
    for r in range(rows):
        for c in range(cols):
            u = ix(r, c)
            for dr, dc in knight:
                nr, nc = r + dr, c + dc
                if 0 <= nr < rows and 0 <= nc < cols:
                    add_e(u, ix(nr, nc))
    for i in range(99):
        add_e(i, (i * 11 + 19) % 99)
        add_e(i, (i * 7 + 23) % 99)
    hub = ix(rows // 2, cols // 2)
    add_e(hub, 99)
    edge_list = [[a, b] for a, b in sorted(edges_set)]
    return {"n": 100, "edges": edge_list, "start_ids": {99: 1}}


if __name__ == "__main__":
    import time
    graph = _build_demo_graph_100_complex()
    ebc_simple = EBCSimpleAlgorithm(graph)
    start = time.time()
    unvisited_edge_num, repe, result = ebc_simple.my_algorithm()
    cost = time.time() - start
    print(cost, unvisited_edge_num, repe, result)