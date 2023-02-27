import sys
import os
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

class DataGenerator:

    def __init__(self, params_filename='parameters.yaml', dataset_count=2500, num_gaussians=5, num_robots=15, new_sz=128, num_steps_per_dataset=1, stable_dataset_count=10, num_neighbor_robots=3):
        self.params_ = pyCoverageControl.Parameters(params_filename)
        self.dataset_count = dataset_count
        self.num_gaussians = num_gaussians
        self.num_robots = num_robots
        self.new_sz = new_sz
        self.num_steps_per_dataset = num_steps_per_dataset
        self.stable_dataset_count = stable_dataset_count
        self.num_neighbor_robots = num_neighbor_robots

        self.compression_ratio = self.params_.pLocalMapSize/self.new_sz

        self.coverage_count = 0
        self.coverage_maps = torch.empty(self.dataset_count, self.new_sz, self.new_sz)

        self.relative_positions = torch.empty(dataset_count, self.num_neighbor_robots * 2) # Positions of robots (x, y) per robot

        self.torch_coverage_features = torch.empty(self.dataset_count, 7)

        self.env = CoverageSystem(self.params_, self.num_gaussians, self.num_robots)
        self.oracle = OracleGlobalOffline(self.params_, self.num_robots, self.env)


    def Step(self):
        cont_flag = self.oracle.Step();
        actions = self.oracle.GetActions()
        error_flag = self.env.StepActions(actions)
        return cont_flag, error_flag

    def StepSave(self):
        robot_positions = self.env.GetRobotPositions()
        voronoi_features = self.env.GetLocalVoronoiFeatures()
        for i in range(0, self.num_robots):
            local_map = self.env.GetRobotLocalMap(i)
            lmap = cv2.resize(local_map, dsize=(self.new_sz,self.new_sz), interpolation=cv2.INTER_AREA)
            self.coverage_maps[self.coverage_count] = torch.tensor(lmap)

            relative_pos_neighboring_robots = self.env.GetRobotsInCommunication(i)
            for j in range(0, self.num_neighbor_robots):
                if j < len(relative_pos_neighboring_robots):
                    self.relative_positions[self.coverage_count, j * 2] = relative_pos_neighboring_robots[j][0]
                    self.relative_positions[self.coverage_count, j * 2 + 1] = relative_pos_neighboring_robots[j][1]
                else:
                    self.relative_positions[self.coverage_count, j * 2] = 0
                    self.relative_positions[self.coverage_count, j * 2 + 1] = 0

            self.torch_coverage_features[self.coverage_count] = torch.tensor(voronoi_features[i])

            self.coverage_count = self.coverage_count + 1
            if not(self.coverage_count < self.dataset_count):
                break

        [cont_flag, error_flag] = self.Step()
        goals = self.oracle.GetGoals()
        actions = self.oracle.GetActions()
        return cont_flag

    def GenerateDataset(self):
        while self.coverage_count < self.dataset_count:
            print("New environment")
            num_steps = 0
            self.env = CoverageSystem(self.params_, self.num_gaussians, self.num_robots)
            self.oracle = OracleGlobalOffline(self.params_, self.num_robots, self.env)

            cont_flag = True
            while num_steps < math.floor(self.params_.pEpisodeSteps/self.num_steps_per_dataset):
                for i in range(0, self.num_steps_per_dataset - 1):
                    [cont_flag, error_flag] = self.Step()
                    if cont_flag == False:
                        break
                if cont_flag == False:
                    break

                cont_flag = self.StepSave()
                if not(self.coverage_count < self.dataset_count):
                    break
                num_steps = num_steps + 1
                print(str(self.coverage_count), str(num_steps))

    def NormalizeTensor(self, tensor_data):
        mean = tensor_data.mean(dim=0)
        std = tensor_data.std(dim=0)
        normalized_tensor_data = (tensor_data - mean)/std
        return mean, std, normalized_tensor_data

    def SaveDatasetSubset(self, dir_name='gnn/train', start_idx=0, end_idx=0):
        torch.save(self.coverage_maps[start_idx:end_idx].clone(), dir_name + '/coverage_maps.pt')
        torch.save(self.relative_positions[start_idx:end_idx].clone(), dir_name + '/relative_positions.pt')
        torch.save(self.normalized_torch_coverage_features[start_idx:end_idx].clone(), dir_name + '/coverage_features.pt')

    def SaveDataset(self, dir_name='gnn'):
        if not os.path.exists(dir_name):
            os.makedirs(dir_name)
        if not os.path.exists(dir_name + '/train'):
            os.makedirs(dir_name + '/train')
        if not os.path.exists(dir_name + '/val'):
            os.makedirs(dir_name + '/val')
        if not os.path.exists(dir_name + '/test'):
            os.makedirs(dir_name + '/test')

        coverage_features_mean, coverage_features_std, self.normalized_torch_coverage_features = self.NormalizeTensor(self.torch_coverage_features)
        torch.save(coverage_features_mean, dir_name + '/coverage_features_mean.pt')
        torch.save(coverage_features_std, dir_name + '/coverage_features_std.pt')

        relative_positions_mean, relative_positions_std, self.normalized_relative_positions = self.NormalizeTensor(self.relative_positions)
        torch.save(relative_positions_mean, dir_name + '/relative_positions_mean.pt')
        torch.save(relative_positions_std, dir_name + '/relative_positions_std.pt')

        val_idx = int(0.7 * self.dataset_count)
        test_idx = val_idx + int(0.2 * self.dataset_count)
        self.SaveDatasetSubset(dir_name + '/train', 0, val_idx)
        self.SaveDatasetSubset(dir_name + '/val', val_idx, test_idx)
        self.SaveDatasetSubset(dir_name + '/test', test_idx, self.dataset_count)

if __name__ == '__main__':
    params_filename = 'parameters.yaml'
    dataset_count = 1000
    num_gaussians = 5
    num_robots = 15

    gen = DataGenerator(params_filename, dataset_count, num_gaussians, num_robots)
    gen.GenerateDataset()
    gen.SaveDataset('cnn_NoComms/')
