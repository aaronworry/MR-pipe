import sys
from collections import defaultdict
import numpy as np


class _Multigraph:
    def __init__(self, n):
        self.n = n
        self.adj = [defaultdict(int) for _ in range(n)]

    def add_undirected(self, u, v, c=1):
        if u == v or c <= 0:
            return
        self.adj[u][v] += c
        self.adj[v][u] += c

    def copy(self):
        g = _Multigraph(self.n)
        for u in range(self.n):
            for v, c in self.adj[u].items():
                if u <= v:
                    g.adj[u][v] = self.adj[u][v]
                    g.adj[v][u] = self.adj[v][u]
        return g

    def remove_one(self, u, v):
        self.adj[u][v] -= 1
        self.adj[v][u] -= 1
        if self.adj[u][v] <= 0:
            del self.adj[u][v]
            del self.adj[v][u]

    def first_neighbor(self, u):
        for v, c in self.adj[u].items():
            if c > 0:
                return v
        return None


def _euler_edge_sequence(multigraph):
    """
    Repeated Hierholzer on each connected Eulerian component.
    Returns a flat list of undirected edges (u, v) covering every edge copy once.
    """
    g = multigraph.copy()
    all_edges = []
    while True:
        start = None
        for u in range(g.n):
            if g.first_neighbor(u) is not None:
                start = u
                break
        if start is None:
            break
        stack = [start]
        circuit = []
        while stack:
            v = stack[-1]
            w = g.first_neighbor(v)
            if w is not None:
                g.remove_one(v, w)
                stack.append(w)
            else:
                circuit.append(stack.pop())
        circuit.reverse()
        all_edges.extend(_edges_from_vertex_walk(circuit))
    return all_edges


def _edges_from_vertex_walk(walk):
    if len(walk) < 2:
        return []
    return [(walk[i], walk[i + 1]) for i in range(len(walk) - 1)]


def _path_from_edges(edge_list):
    if not edge_list:
        return []
    out = [edge_list[0][0]]
    for a, b in edge_list:
        out.append(b)
    return out


def _concat_vertex_paths(prefix, main):
    if not prefix:
        return list(main)
    if not main:
        return list(prefix)
    if prefix[-1] == main[0]:
        return prefix + list(main[1:])
    return prefix + list(main)


class KCPPAlgorithm:
    """
    k-route Chinese Postman (heuristic): CPP augmentation on the undirected graph
    (minimum-cost pairing of odd-degree vertices via shortest paths), one Eulerian
    circuit on the augmented multigraph, then partition of circuit edges among
    k robots with shortest-path connectors from each robot start.
    """

    def __init__(self, graph, robots):
        self.robots = robots
        self.Graph = graph
        self.node = self.Graph.node
        self.start_ids = None
        self.get_start_ids(self.robots)
        self.k = len(self.robots)
        self.graph = [list(elem) for elem in self.Graph.get_edges()]

    def get_start_ids(self, robots):
        start_ids = {}
        for robot in robots:
            for i in range(self.node):
                if np.array_equal(self.Graph.graph.vs[i]["position"], robot.position):
                    if self.Graph.graph.vs[i]["id"] not in start_ids:
                        start_ids[self.Graph.graph.vs[i]["id"]] = 1
                    else:
                        start_ids[self.Graph.graph.vs[i]["id"]] += 1
        self.start_ids = start_ids

    def _sp_vertices(self, a, b):
        paths = self.Graph.graph.get_shortest_paths(
            a,
            to=b,
            weights=self.Graph.graph.es["weight"],
            output="vpath",
        )
        p = paths[0] if paths else []
        return p if p else [a]

    def _odd_vertices(self, mg):
        odds = []
        for u in range(mg.n):
            d = sum(mg.adj[u].values())
            if d % 2 == 1:
                odds.append(u)
        return odds

    def _min_matching_augment(self, mg, odds):
        m = len(odds)
        if m == 0:
            return

        dist_path = {}
        for i in range(m):
            for j in range(i + 1, m):
                a, b = odds[i], odds[j]
                p = self._sp_vertices(a, b)
                dist_path[(i, j)] = (self.Graph.get_length_graph(p), p)
                dist_path[(j, i)] = dist_path[(i, j)]

        def best_pairing(indices):
            if len(indices) <= 2:
                if len(indices) == 0:
                    return 0, []
                if len(indices) == 2:
                    i, j = indices[0], indices[1]
                    cost, p = dist_path[(i, j)]
                    return cost, [p]
                return 0, []
            i0 = indices[0]
            best_c = sys.maxsize
            best_paths = None
            for j in range(1, len(indices)):
                i1 = indices[j]
                rest = [indices[t] for t in range(len(indices)) if t not in (0, j)]
                c01, p01 = dist_path[(i0, i1)]
                cr, pr = best_pairing(rest)
                if c01 + cr < best_c:
                    best_c = c01 + cr
                    best_paths = [p01] + pr
            return best_c, best_paths

        def greedy_pairing(indices):
            odds_local = list(indices)
            all_paths = []
            while odds_local:
                v = odds_local.pop(0)
                if not odds_local:
                    break
                best_j = 0
                best_c = sys.maxsize
                best_p = None
                for j, u in enumerate(odds_local):
                    iv = odds.index(v)
                    iu = odds.index(u)
                    ia, ib = min(iv, iu), max(iv, iu)
                    c, p = dist_path[(ia, ib)]
                    if c < best_c:
                        best_c = c
                        best_j = j
                        best_p = p
                u = odds_local.pop(best_j)
                all_paths.append(best_p)
            return all_paths

        if m <= 12:
            idx = list(range(m))
            _, pair_paths = best_pairing(idx)
        else:
            pair_paths = greedy_pairing(odds)

        for p in pair_paths:
            for t in range(len(p) - 1):
                mg.add_undirected(p[t], p[t + 1], 1)

    def solve(self):
        n = self.node
        mg = _Multigraph(n)
        for e in self.graph:
            mg.add_undirected(e[0], e[1], 1)

        odds = self._odd_vertices(mg)
        self._min_matching_augment(mg, odds)

        edge_seq = _euler_edge_sequence(mg)

        starts = []
        for key, value in self.start_ids.items():
            for _ in range(value):
                starts.append(int(key))

        while len(starts) < self.k:
            starts.append(starts[-1] if starts else 0)
        starts = starts[: self.k]

        walks = []
        m = len(edge_seq)
        if m == 0:
            for i in range(self.k):
                s = starts[i]
                walks.append(
                    {
                        "path": [s],
                        "length": 0,
                        "count": 1,
                    }
                )
        else:
            for i in range(self.k):
                lo = i * m // self.k
                hi = (i + 1) * m // self.k
                chunk = edge_seq[lo:hi]
                sub = _path_from_edges(chunk)
                dep = starts[i]
                if not sub:
                    sp = self._sp_vertices(dep, dep)
                    full = sp if sp else [dep]
                else:
                    sp0 = self._sp_vertices(dep, sub[0])
                    full = _concat_vertex_paths(sp0, sub)
                ln = self.Graph.get_length_graph(full)
                walks.append(
                    {
                        "path": full,
                        "length": ln,
                        "count": len(full),
                    }
                )

        unvisited_edge_num, sum_visited_edge = self.checkResult(walks)
        qq = len(self.graph) - unvisited_edge_num
        repe = (sum_visited_edge - qq) / qq if qq else 0.0
        return unvisited_edge_num, repe, walks

    def checkResult(self, paths):
        temp_mat = np.zeros((self.Graph.node, self.Graph.node))
        for item in paths:
            path = item["path"]
            for i in range(len(path) - 1):
                if path[i] != path[i + 1]:
                    temp_mat[path[i]][path[i + 1]] += 1
                    temp_mat[path[i + 1]][path[i]] += 1
        sum_visited_edge = np.sum(temp_mat) / 2
        covered = [0] * len(self.graph)
        for item in paths:
            path = item["path"]
            for idx_e, edge in enumerate(self.graph):
                if self._path_covers_edge(path, edge):
                    covered[idx_e] = 1
        unvisited = sum(1 for x in covered if x == 0)
        return unvisited, sum_visited_edge

    def _path_covers_edge(self, path, edge):
        a, b = edge[0], edge[1]
        for i in range(len(path) - 1):
            u, v = path[i], path[i + 1]
            if (u == a and v == b) or (u == b and v == a):
                return True
        return False
