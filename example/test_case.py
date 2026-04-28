import sys
import time
sys.path.append("..")

from env.env_base import Env
import numpy as np
from algorithm.bfsAlgorithm import BFSAlgorithm
from algorithm.ebcAlgorithm import EBCAlgorithm
from algorithm.dfsAlgorithm import DFSAlgorithm
from algorithm.ilpAlgorithm import ILPAlgorithm
from algorithm.kCPPAlgorithm import KCPPAlgorithm

env = Env(dt = 0.8, pipe_path="../maps/case30v_6r_3layer.yaml", dim=3)
start = time.time()

# DFS-based
# alg = DFSAlgorithm(env.graph, env.robots)
# unvisited_num, repetition, walks = alg.find_optimize_solution()

# EBC
alg = EBCAlgorithm(env.graph, env.robots)
unvisited_num, repetition, walks = alg.my_algorithm()

# BFS-based
# alg = BFSAlgorithm(env.graph, env.robots)
# unvisited_num, repetition, walks = alg.solve()

# ILP
# alg = ILPAlgorithm(env.graph, env.robots)
# unvisited_num, repetition, walks = alg.solve()

# KCPP
# alg = KCPPAlgorithm(env.graph, env.robots)
# unvisited_num, repetition, walks = alg.solve()

cost = time.time() - start
print(cost, unvisited_num, repetition, walks)


start2 = time.time()
env.path_planning(walks)
while not len(env.robot_finihsed_set) == env.robot_num:
    env.step_path()
print(time.time() - start2)