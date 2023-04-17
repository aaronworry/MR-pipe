import sys
sys.path.append("..")

from env.env_base import Env
import numpy as np
from algorithm.exhaustiveAlgorithm import ExhaustiveAlgorithm
from algorithm.heuristicAlgorithm import HeuristicAlgorithm
from algorithm.exhaustiveSpaceToTime import ExhaustiveSpaceToTime

env = Env(dt = 0.8, pipe_path="../maps/case0.yaml", dim=3)


# alg = ExhaustiveAlgorithm(env.graph, env.robots)
# walks = alg.find_optimize_solution()    

alg = HeuristicAlgorithm(env.graph, env.robots)
walks = alg.my_algorithm()

# alg = ExhaustiveSpaceToTime(env.graph, env.robots)
# walks = alg.solve()

print(walks)
env.path_planning(walks)    
while not len(env.robot_finihsed_set) == env.robot_num:
    env.step_path()
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    '''
    for i in range(300):

        des_vel = env.car.cal_des_vel()
        env.car.move_forward(des_vel)

        env.render()
    

    env.show()
    '''
    
    