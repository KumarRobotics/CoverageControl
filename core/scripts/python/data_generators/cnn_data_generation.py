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
from torchvision.transforms import Resize
import torch
import cv2
from scipy.ndimage import gaussian_filter
import matplotlib.pylab as plt
import seaborn as sns

params_ = pyCoverageControl.Parameters('parameters.yaml')

new_sz = 128
compression_ratio = params_.pLocalMapSize/new_sz
resize = Resize((new_sz,new_sz))

num_gaussians = 5
num_robots = 20

coverage_count = 0
exp_count = 0
robot_id = 0

dataset_count = 250000
batch_size = 1


# total_dataset_count = dataset_count + dummy_count * num_robots
coverage_maps = torch.empty(dataset_count, 2, new_sz, new_sz)
# exploration_maps = torch.empty(dataset_count, new_sz, new_sz)

torch_coverage_features = torch.empty(dataset_count, 7)
# torch_exploration_features = torch.empty(dataset_count, 2 * params_.pNumFrontiers)

def Step():
    global env, oracle
    cont_flag = oracle.Step();
    actions = oracle.GetActions()
    positions = env.GetRobotPositions()
    error_flag = env.StepActions(actions)
    return cont_flag, error_flag

def StepSave():
    global env, oracle, coverage_count, exp_count
    global resize, kernel
    global heatmap_x, heatmap_y
    global coverage_maps, exploration_maps

    [cont_flag, error_flag] = Step()
    # env.RecordPlotData()

    robot_positions=env.GetRobotPositions()
    goals = oracle.GetGoals()
    actions = oracle.GetActions()

    voronoi_features = env.GetLocalVoronoiFeatures()

    for i in range(0, num_robots):
        local_map = env.GetRobotLocalMap(i)
        lmap = cv2.resize(local_map, dsize=(new_sz,new_sz), interpolation=cv2.INTER_AREA)
        coverage_maps[coverage_count, 0] = torch.tensor(lmap)

        comm_map = env.GetCommunicationMap(i)
        comm_map = torch.tensor(gaussian_filter(comm_map, sigma=(3,3), order=0))
        comm_map = cv2.resize(np.array(comm_map), dsize=(new_sz, new_sz), interpolation=cv2.INTER_AREA)
        coverage_maps[coverage_count, 1] = torch.tensor(comm_map)

        torch_coverage_features[coverage_count] = torch.tensor(voronoi_features[i])
        coverage_count = coverage_count + 1
        if not(coverage_count < dataset_count):
            break
    return cont_flag

while coverage_count < dataset_count:
    print("New environment")
    num_steps = 0
    env = CoverageSystem(params_, num_gaussians, num_robots)
    oracle = OracleGlobalOffline(params_, num_robots, env)
    # env.PlotWorldMap(".")

    cont_flag = True
    while num_steps < math.floor(params_.pEpisodeSteps/batch_size):
        for i in range(0, batch_size - 1):
            [cont_flag, error_flag] = Step()
            if cont_flag == False:
                break
        if cont_flag == False:
            break

        cont_flag = StepSave()

        num_steps = num_steps + 1
        print(str(coverage_count), str(exp_count), str(num_steps))
        if not(coverage_count < dataset_count):
            break
    # for i in range(0, dummy_count):
    #     cont_flag = StepSave()
    # env.RenderRecordedMap(".", "vid.mp4")
    # env.PlotSystemMap(".", 0)
    # exit()


coverage_features_mean = torch_coverage_features.mean(dim=0)
coverage_features_std = torch_coverage_features.std(dim=0)
normalized_torch_coverage_features = (torch_coverage_features - coverage_features_mean)/coverage_features_std

torch.save(coverage_features_mean, 'cnn/coverage_features_mean.pt')
torch.save(coverage_features_std, 'cnn/coverage_features_std.pt')

val_idx = int(0.7 * dataset_count)
test_idx = val_idx + int(0.2 * dataset_count)

torch.save(coverage_maps[0:val_idx].clone(), 'cnn/train/coverage_maps.pt')
torch.save(normalized_torch_coverage_features[0:val_idx].clone(), 'cnn/train/coverage_features_targets.pt')

torch.save(coverage_maps[val_idx:test_idx].clone(), 'cnn/val/coverage_maps.pt')
torch.save(normalized_torch_coverage_features[val_idx:test_idx].clone(), 'cnn/val/coverage_features_targets.pt')

torch.save(coverage_maps[test_idx:].clone(), 'cnn/test/coverage_maps.pt')
torch.save(normalized_torch_coverage_features[test_idx:].clone(), 'cnn/test/coverage_features_targets.pt')
