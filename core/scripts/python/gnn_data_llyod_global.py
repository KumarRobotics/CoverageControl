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
def write_npz(iRobot):
    np.savez_compressed('../../../data/gnn_data/data_' + f'{(count * num_robots + iRobot):07d}' + '.npz', local_map = env.GetRobotLocalMap(iRobot), communication_map = env.GetCommunicationMap(iRobot), label=np.concatenate((env.GetVoronoiCell(iRobot).centroid, [env.GetVoronoiCell(iRobot).mass])))

dataset_count = 100
while count < dataset_count:
    num_steps = 0
    env = CoverageSystem(params_, num_gaussians, num_robots)

    oracle = OracleGlobalOffline(params_, num_robots, env)

    cont_flag = True
    while num_steps < params_.pEpisodeSteps and count < dataset_count:
        cont_flag = oracle.Step()
        actions = oracle.GetActions()
        local_features = env.GetLocalVoronoiFeatures()
        if(not cont_flag):
            break
        positions = env.GetRobotPositions()
        # pool = Pool(num_robots)
        # pool.map(write_npz, range(0, num_robots))
        num_steps = num_steps + 1
        count = count + 1
