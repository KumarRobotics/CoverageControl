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

params_ = pyCoverageControl.Parameters('params/parameters.yaml')

new_sz = 64
map_data_shape = [1, new_sz, new_sz]
compression_ratio = params_.pLocalMapSize/new_sz
resize = Resize((new_sz,new_sz))

num_gaussians = 10
num_robots = 15

count = 0
robot_id = 0

dataset_count = 1000 * num_robots
batch_size = 5
dummy_count = 10

total_dataset_count = dataset_count + dummy_count * num_robots
coverage_maps = torch.empty(total_dataset_count, new_sz, new_sz)
exploration_maps = torch.empty(total_dataset_count, new_sz, new_sz)

torch_voronoi_features = torch.empty(total_dataset_count, 3)
torch_exploration_features = torch.empty(total_dataset_count, 2 * params_.pNumFrontiers)

def Step():
    global env, oracle
    cont_flag = oracle.Step();
    actions = oracle.GetActions()
    positions = env.GetRobotPositions()
    error_flag = env.StepActions(actions)
    return cont_flag, error_flag

def StepSave():
    global env, oracle, count
    global resize, kernel
    global heatmap_x, heatmap_y
    global coverage_maps, exploration_maps

    voronoi_features = env.GetRobotVoronoiFeatures()
    exploration_features = env.GetRobotExplorationFeatures()

    for i in range(0, num_robots):
        lmap = cv2.resize(env.GetRobotLocalMap(i), dsize=(new_sz,new_sz), interpolation=cv2.INTER_AREA)
        # coverage_maps = torch.cat((coverage_maps, torch.tensor([lmap])), 0)
        coverage_maps[count] = torch.tensor(lmap)

        explmap = env.GetRobotExplorationMap(i).astype(np.float32)
        explmap = cv2.resize(explmap, dsize=(new_sz,new_sz), interpolation=cv2.INTER_AREA)
        # exploration_maps = torch.cat((exploration_maps, torch.tensor([explmap])), 0)
        exploration_maps[count] = torch.tensor(explmap)

        torch_voronoi_features[count] = torch.tensor(voronoi_features[i])
        torch_exploration_features[count] = torch.tensor(exploration_features[i])

        count = count + 1


    robot_positions=env.GetRobotPositions()

    [cont_flag, error_flag] = Step()

    goals = oracle.GetGoals()
    actions = oracle.GetActions()
    robot_status = oracle.GetRobotStatus()

    return cont_flag

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
        if count >= dataset_count:
            break;
    for i in range(0, dummy_count):
        cont_flag = StepSave()
        # [cont_flag, error_flag] = Step()

# local_maps_torch = torch.tensor(coverage_maps)
# exploration_maps_torch = torch.tensor(exploration_maps)
# print(local_maps_torch.shape)
# print(exploration_maps_torch.shape)

map_shape = coverage_maps.shape

h_vals = torch.linspace(1.0, -1.0, map_shape[1]+1)
h_vals = (h_vals[1:] + h_vals[:-1])/2
w_vals = torch.linspace(-1.0, 1.0, map_shape[2]+1)
w_vals = (w_vals[1:] + w_vals[:-1])/2
heatmap_x = torch.stack([h_vals]*map_shape[2], axis=1)/100
heatmap_y = torch.stack([w_vals]*map_shape[1], axis=0)/100
heatmap_x = torch.stack([heatmap_x]*map_shape[0], dim=0)
heatmap_y = torch.stack([heatmap_y]*map_shape[0], dim=0)


coverage_maps_obs = torch.stack([coverage_maps, heatmap_x, heatmap_y], dim=1)
exploration_map_obs = torch.stack([exploration_maps, heatmap_x, heatmap_y], dim=1)
print(coverage_maps_obs.shape)
print(exploration_map_obs.shape)
torch.save(coverage_maps_obs, 'coverage_maps.pt')
torch.save(exploration_map_obs, 'exploration_maps.pt')
torch.save(torch_voronoi_features, 'coverage_targets.pt')
torch.save(torch_exploration_features, 'exploration_targets.pt')
