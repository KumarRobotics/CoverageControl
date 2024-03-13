#  This file is part of the CoverageControl library
#
#  Author: Saurav Agarwal
#  Contact: sauravag@seas.upenn.edu, agr.saurav1@gmail.com
#  Repository: https://github.com/KumarRobotics/CoverageControl
#
#  Copyright (c) 2024, Saurav Agarwal
#
#  The CoverageControl library is free software: you can redistribute it and/or
#  modify it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or (at your
#  option) any later version.
#
#  The CoverageControl library is distributed in the hope that it will be
#  useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
#  Public License for more details.
#
#  You should have received a copy of the GNU General Public License along with
#  CoverageControl library. If not, see <https://www.gnu.org/licenses/>.

# @file data_generation.py
# This file contains the code to generate a dataset for learning
# Prefer using simple_data_generation.py for generating a dataset
# 

# DataDir = "~/CoverageControl_ws/src/CoverageControl/" # Absolute path to the root of the repository
# EnvironmentConfig = "params/coverage_control_params.toml" # Relative to DataDir
#
# NumDataset = 1000
#
# # Number of steps to take before data is stores
# # This helps in creating a more diverse dataset
# EveryNumSteps = 5
#
# # The robots stop moving once the algorithm has converged
# # Having some of these converged steps can help in stabilizing robot actions
# ConvergedDataRatio = 0.25
#
# # Resizing of maps and Sparsification of tensors are triggered every TriggerPostProcessing dataset
# # This should be set based on RAM resources available on the system
# TriggerPostProcessing = 100
#
# CNNMapSize = 32
# SaveAsSparseQ = true
# NormalizeQ = true
#
# [DataSetSplit]
# TrainRatio = 0.7
# ValRatio =  0.2

import os
import sys
import torch
import math
import pathlib
import datetime

import coverage_control
from coverage_control import IOUtils
from coverage_control import CoverageSystem
from coverage_control.algorithms import ClairvoyantCVT as CoverageAlgorithm
from coverage_control.nn import CoverageEnvUtils

class DatasetGenerator():
    def __init__(self, config_file, append_dir=None):

        self.config = IOUtils.load_toml(config_file)
        self.data_dir = IOUtils.sanitize_path(self.config['DataDir'])
        self.dataset_dir = self.data_dir + '/data/'
        if append_dir is not None:
            self.dataset_dir += append_dir

        if not pathlib.Path(self.data_dir).exists():
            print(f'{self.data_dir} does not exist')
            exit()

        self.dataset_dir_path = pathlib.Path(self.dataset_dir)
        if not self.dataset_dir_path.exists():
            os.makedirs(self.dataset_dir)

        env_config_file = IOUtils.sanitize_path(self.config["EnvironmentConfig"])
        env_config_file = pathlib.Path(env_config_file)
        if not env_config_file.exists():
            print(f'{env_config_file} does not exist')
            exit()

        self.env_params = coverage_control.Parameters(env_config_file.as_posix())

        # Initialize variables
        self.dataset_count = 0
        self.env_count = 0
        self.trigger_count = 0
        self.trigger_start_idx = 0

        self.num_dataset = self.config["NumDataset"]
        self.num_robots = self.env_params.pNumRobots
        self.comm_range = self.env_params.pCommunicationRange
        self.resolution = self.env_params.pResolution
        self.cnn_map_size = self.config["CNNMapSize"]
        self.every_num_step = self.config["EveryNumSteps"]
        self.trigger_size = self.config["TriggerPostProcessing"]
        self.converged_data_ratio = self.config["ConvergedDataRatio"]
        if self.trigger_size == 0 or self.trigger_size > self.num_dataset:
            self.trigger_size = self.num_dataset

        if torch.cuda.is_available():
            self.device = torch.device("cuda")
        else:
            self.device = torch.device("cpu")

        # Initialize tensors
        self.actions = torch.zeros((self.num_dataset, self.num_robots, 2))
        self.robot_positions = torch.zeros((self.num_dataset, self.num_robots, 2))
        self.raw_local_maps = torch.zeros((self.trigger_size, self.num_robots, self.env_params.pLocalMapSize, self.env_params.pLocalMapSize))
        self.raw_obstacle_maps = torch.zeros((self.trigger_size, self.num_robots, self.env_params.pLocalMapSize, self.env_params.pLocalMapSize))
        self.local_maps = torch.zeros((self.num_dataset, self.num_robots, self.cnn_map_size, self.cnn_map_size))
        self.obstacle_maps = torch.zeros((self.num_dataset, self.num_robots, self.cnn_map_size, self.cnn_map_size))
        self.comm_maps = torch.zeros((self.num_dataset, self.num_robots, 2, self.cnn_map_size, self.cnn_map_size))
        self.coverage_features = torch.zeros((self.num_dataset, self.num_robots, 7))
        self.edge_weights = torch.zeros((self.num_dataset, self.num_robots, self.num_robots))

        # Write metrics
        self.metrics_file = self.dataset_dir_path / 'metrics.txt'
        self.metrics = open(self.metrics_file, 'w')
        # Get current time
        start_time = datetime.datetime.now()
        self.metrics.write('Time: ' + str(datetime.datetime.now()) + '\n')
        self.metrics.write('Dataset directory: ' + self.dataset_dir + '\n')
        self.PrintTensorSizes()
        self.PrintTensorSizes(self.metrics)
        self.metrics.flush()

        self.RunDataGeneration()
        self.SaveDataset()
        end_time = datetime.datetime.now()
        self.metrics.write('Time: ' + str(datetime.datetime.now()) + '\n')
        self.metrics.write('Total time: ' + str(end_time - start_time) + '\n')
        self.metrics.close()


    def RunDataGeneration(self):
        num_non_converged_env = 0
        while self.dataset_count < self.num_dataset:
            self.env = CoverageSystem(self.env_params, self.env_params.pNumFeatures, self.num_robots)
            self.alg = CoverageAlgorithm(self.env_params, self.num_robots, self.env)
            self.env_count += 1
            print('Environment: ' + str(self.env_count))
            num_steps = 0
            is_converged = False
            while num_steps < self.env_params.pEpisodeSteps and not is_converged and self.dataset_count < self.num_dataset:
                if num_steps % self.every_num_step == 0:
                    is_converged = self.StepWithSave()
                else:
                    is_converged = self.StepWithoutSave()
                num_steps += 1
            if num_steps == self.env_params.pEpisodeSteps:
                num_non_converged_env += 1
                print('Non-converged environment: ' + str(num_non_converged_env))

            num_converged_data = math.ceil(self.converged_data_ratio * num_steps / self.every_num_step)
            converged_data_count = 0
            while converged_data_count < num_converged_data and self.dataset_count < self.num_dataset:
                self.StepWithSave()
                converged_data_count += 1

    def StepWithSave(self):
        self.alg.ComputeActions()
        converged = self.alg.IsConverged()
        actions = self.alg.GetActions()
        count = self.dataset_count
        self.actions[count] = CoverageEnvUtils.to_tensor(actions)
        self.robot_positions[count] = CoverageEnvUtils.get_robot_positions(self.env)
        self.coverage_features[count] = CoverageEnvUtils.get_voronoi_features(self.env)
        self.raw_local_maps[self.trigger_count] = CoverageEnvUtils.get_raw_local_maps(self.env, self.env_params)
        self.raw_obstacle_maps[self.trigger_count] = CoverageEnvUtils.get_raw_obstacle_maps(self.env, self.env_params)
        self.comm_maps[count] = CoverageEnvUtils.get_communication_maps(self.env, self.env_params, self.cnn_map_size)
        self.edge_weights[count] = CoverageEnvUtils.get_weights(self.env, self.env_params)
        self.dataset_count += 1
        if self.dataset_count % 100 == 0:
            print(f'Dataset: {self.dataset_count}/{self.num_dataset}')

        self.trigger_count += 1
        if self.trigger_count == self.trigger_size:
            self.TriggerPostProcessing()
            self.trigger_count = 0

        error_flag = self.env.StepActions(actions)
        return converged or error_flag

    def TriggerPostProcessing(self):
        if self.trigger_start_idx > self.num_dataset -1:
            return
        trigger_end_idx = min(self.num_dataset, self.trigger_start_idx + self.trigger_size)
        raw_local_maps = self.raw_local_maps[0:trigger_end_idx - self.trigger_start_idx]
        raw_local_maps = raw_local_maps.to(self.device)
        resized_local_maps = CoverageEnvUtils.resize_maps(raw_local_maps, self.cnn_map_size)
        self.local_maps[self.trigger_start_idx:trigger_end_idx] = resized_local_maps.view(-1, self.num_robots, self.cnn_map_size, self.cnn_map_size).cpu().clone()

        raw_obstacle_maps = self.raw_obstacle_maps[0:trigger_end_idx - self.trigger_start_idx]
        raw_obstacle_maps = raw_obstacle_maps.to(self.device)
        resized_obstacle_maps = CoverageEnvUtils.resize_maps(raw_obstacle_maps, self.cnn_map_size)
        self.obstacle_maps[self.trigger_start_idx:trigger_end_idx] = resized_obstacle_maps.view(-1, self.num_robots, self.cnn_map_size, self.cnn_map_size).cpu().clone()

        self.trigger_start_idx = trigger_end_idx

    def NormalizeTensor(self, tensor):
        tensor_mean = tensor.mean(dim=[0, 1])
        tensor_std = tensor.std(dim=[0, 1])
        tensor = (tensor - tensor_mean) / tensor_std
        return tensor, tensor_mean, tensor_std

    def NormalizeCommunicationMaps(self):
        min_val = self.comm_maps.min()
        max_val = self.comm_maps.max()
        range_val = max_val - min_val
        self.comm_maps = (self.comm_maps - min_val) / range_val
        print('Communication map min: ' + str(min_val))
        print('Communication map max: ' + str(max_val))
        return min_val, range_val

    def SaveTensor(self, tensor, name, as_sparse=False):
        tensor = tensor.cpu()
        train_tensor = tensor[0:self.train_size].clone()
        validation_tensor = tensor[self.train_size:self.train_size + self.validation_size].clone()
        test_tensor = tensor[self.train_size + self.validation_size:].clone()
        if as_sparse:
            train_tensor = train_tensor.to_sparse()
            validation_tensor = validation_tensor.to_sparse()
            test_tensor = test_tensor.to_sparse()

        dataset_dir_path = pathlib.Path(self.dataset_dir)
        torch.save(train_tensor, dataset_dir_path / 'train/' / name)
        torch.save(validation_tensor, dataset_dir_path / 'val/' / name)
        torch.save(test_tensor, dataset_dir_path / 'test/' / name)

    def SaveDataset(self):
        as_sparse = self.config['SaveAsSparseQ']
        self.train_size = int(self.num_dataset * self.config['DataSetSplit']['TrainRatio'])
        self.validation_size = int(self.num_dataset * self.config['DataSetSplit']['ValRatio'])
        self.test_size = self.num_dataset - self.train_size - self.validation_size

        # Make sure the folder exists
        if not os.path.exists(self.dataset_dir + '/train'):
            os.makedirs(self.dataset_dir + '/train')
        if not os.path.exists(self.dataset_dir + '/val'):
            os.makedirs(self.dataset_dir + '/val')
        if not os.path.exists(self.dataset_dir + '/test'):
            os.makedirs(self.dataset_dir + '/test')

        self.SaveTensor(self.robot_positions, 'robot_positions.pt')
        self.SaveTensor(self.local_maps, 'local_maps.pt', as_sparse)
        self.SaveTensor(self.obstacle_maps, 'obstacle_maps.pt', as_sparse)
        self.SaveTensor(self.edge_weights, 'edge_weights.pt', as_sparse)

        # min_val, range_val = self.NormalizeCommunicationMaps()
        self.SaveTensor(self.comm_maps, 'comm_maps.pt', as_sparse)
        # torch.save(min_val, self.dataset_dir / 'comm_maps_min.pt')
        # torch.save(range_val, self.dataset_dir / 'comm_maps_range.pt')

        self.SaveTensor(self.actions, 'actions.pt')
        self.SaveTensor(self.coverage_features, 'coverage_features.pt')

        if self.config['NormalizeQ']:
            normalized_actions, actions_mean, actions_std = self.NormalizeTensor(self.actions)
            coverage_features, coverage_features_mean, coverage_features_std = self.NormalizeTensor(self.coverage_features)
            self.SaveTensor(normalized_actions, 'normalized_actions.pt')
            self.SaveTensor(coverage_features, 'normalized_coverage_features.pt')
            torch.save(actions_mean, self.dataset_dir_path / 'actions_mean.pt')
            torch.save(actions_std, self.dataset_dir_path / 'actions_std.pt')
            torch.save(coverage_features_mean, self.dataset_dir_path / 'coverage_features_mean.pt')
            torch.save(coverage_features_std, self.dataset_dir_path / 'coverage_features_std.pt')


    def StepWithoutSave(self):
        self.alg.ComputeActions()
        converged = self.alg.IsConverged()
        if (self.env.StepActions(self.alg.GetActions())):
            return True
        return converged

    def GetTensorByteSizeMB(self, tensor):
        return (tensor.element_size() * tensor.nelement()) / (1024 * 1024)

    def PrintTensorSizes(self, file=sys.stdout):
        # Set to two decimal places
        print('Tensor sizes:', file=file)
        print('Actions:', self.GetTensorByteSizeMB(self.actions), file=file)
        print('Robot positions:', self.GetTensorByteSizeMB(self.robot_positions), file=file)
        print('Raw local maps:', self.GetTensorByteSizeMB(self.raw_local_maps), file=file)
        print('Raw obstacle maps:', self.GetTensorByteSizeMB(self.raw_obstacle_maps), file=file)
        print('Local maps:', self.GetTensorByteSizeMB(self.local_maps), file=file)
        print('Obstacle maps:', self.GetTensorByteSizeMB(self.obstacle_maps), file=file)
        print('Comm maps:', self.GetTensorByteSizeMB(self.comm_maps), file=file)
        print('Coverage features:', self.GetTensorByteSizeMB(self.coverage_features), file=file)

if __name__ == '__main__':
    config_file = sys.argv[1]
    if len(sys.argv) > 2:
        append_folder = sys.argv[2]
    else:
        append_folder = None
    DatasetGenerator(config_file)
