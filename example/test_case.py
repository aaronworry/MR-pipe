import sys
import time
sys.path.append("..")

from env.env_base import Env
import numpy as np
from algorithm.exhaustiveAlgorithm import ExhaustiveAlgorithm
from algorithm.heuristicAlgorithm import HeuristicAlgorithm
from algorithm.exhaustiveSpaceToTime import ExhaustiveSpaceToTime

env = Env(dt = 0.8, pipe_path="../maps/case5.yaml", dim=3)

start = time.time()
# alg = ExhaustiveAlgorithm(env.graph, env.robots)
# unvisited_num, repetition, walks = alg.find_optimize_solution()    

alg = HeuristicAlgorithm(env.graph, env.robots)
unvisited_num, repetition, walks = alg.my_algorithm()

# alg = ExhaustiveSpaceToTime(env.graph, env.robots)
# unvisited_num, repetition, walks = alg.solve()

cost = time.time() - start
print(cost, unvisited_num, repetition, walks)

env.path_planning(walks)    
while not len(env.robot_finihsed_set) == env.robot_num:
    env.step_path()

    
    