import sys
import math
import time
import numpy as np
from sklearn.metrics import pairwise_distances as pwdist
from scipy.optimize import linear_sum_assignment
import pyCoverageControl # Main library
from pyCoverageControl import Point2 # for defining points
from pyCoverageControl import PointVector # for defining list of points
from pyCoverageControl import CoverageSystem
from pyCoverageControl import OracleGlobalOffline
from multiprocessing import Pool

params_ = pyCoverageControl.Parameters('params/parameters.yaml')

num_gaussians = 100
num_robots = 20

count = 0
robot_id = 0

# Need to modify this function
def write_npz():
    local_maps = []
    communication_maps = []
    exploration_maps = []
    for i in range(0, num_robots):
        local_maps.append(env.GetRobotLocalMap(i))
        communication_maps.append(env.GetCommunicationMap(i))
        exploration_maps.append(env.GetRobotExplorationMap(i))

    np.savez_compressed('../../../data/gnn_data/data_' + f'{(count):07d}' + '.npz',
            robot_positions=env.GetRobotPositions(),
            local_maps = local_maps,
            communication_maps = communication_maps,
            features=env.GetLocalVoronoiFeatures(),
            labels=oracle.GetActions(),
            objective_value=env.GetObjectiveValue()
            )

dataset_count = 200000
while count < dataset_count:
    print("New environment")
    num_steps = 0
    env = CoverageSystem(params_, num_gaussians, num_robots)

    oracle = OracleGlobalOffline(params_, num_robots, env)

    cont_flag = True
    while num_steps < params_.pEpisodeSteps and count < dataset_count:
        cont_flag = oracle.Step()
        # actions = oracle.GetActions()
        # local_features = env.GetLocalVoronoiFeatures()
        if(not cont_flag):
            break
        positions = env.GetRobotPositions()
        # pool = Pool(num_robots)
        # pool.map(write_npz, range(0, num_robots))
        write_npz()
        num_steps = num_steps + 1
        count = count + 1
        print(str(count))
