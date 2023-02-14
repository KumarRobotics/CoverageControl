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
from pyCoverageControl import OracleSimulExploreExploit
from multiprocessing import Pool
from torchvision.transforms import Resize
import torch
import cv2
from scipy.ndimage import gaussian_filter
import matplotlib.pylab as plt
import seaborn as sns
resize = Resize((128,128))

params_ = pyCoverageControl.Parameters()

num_gaussians = 10
num_robots = 15

count = 0
robot_id = 0

cnn_voronoi_count = 0
cnn_explore_count = 0

def Step():
    global env, oracle
    cont_flag = oracle.Step();
    actions = oracle.GetActions()
    positions = env.GetRobotPositions()
    error_flag = env.StepActions(actions)
    return cont_flag, error_flag

def StepSave():
    global env, oracle
    global count
    global cnn_voronoi_count
    global cnn_explore_count
    global resize
    global kernel
    local_maps = []
    communication_maps = []
    exploration_maps = []
    for i in range(0, num_robots):
        lmap = cv2.resize(env.GetRobotLocalMap(i), dsize=(128,128), interpolation=cv2.INTER_AREA)
        local_maps.append(lmap)

        explmap = env.GetRobotExplorationMap(i).astype(np.float32)
        explmap = cv2.resize(explmap, dsize=(128,128), interpolation=cv2.INTER_AREA)
        exploration_maps.append(explmap)

    voronoi_features = env.GetRobotVoronoiFeatures()
    exploration_features = env.GetRobotExplorationFeatures()

    robot_positions=env.GetRobotPositions()

    [cont_flag, error_flag] = Step()

    goals = oracle.GetGoals()
    actions = oracle.GetActions()
    robot_status = oracle.GetRobotStatus()

    count = count + 1
    return cont_flag

dataset_count = 200000
batch_size = 5
while count < dataset_count:
    print("New environment")
    num_steps = 0
    env = CoverageSystem(params_, num_gaussians, num_robots)
    oracle = OracleSimulExploreExploit(params_, num_robots, env)

    cont_flag = True
    while num_steps < math.floor(params_.pEpisodeSteps/batch_size):
        for i in range(0, batch_size - 1):
            [cont_flag, error_flag] = Step()
            if cont_flag == False:
                break;
        if cont_flag == False:
            break;

        cont_flag = StepSave()

        num_steps = num_steps + 1
        print(str(count), str(num_steps))
    for i in range(0, 10):
        [cont_flag, error_flag] = Step()
