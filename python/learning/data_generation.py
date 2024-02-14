import os
import sys
import torch
import datetime
import math

import pyCoverageControl as CoverageControl
from pyCoverageControl import CoverageSystem
from pyCoverageControl import LloydGlobalOnline as CoverageAlgorithm
# from pyCoverageControl import OracleGlobalOffline as CoverageAlgorithm
# from pyCoverageControl import LloydLocalSensorGlobalComm as CoverageAlgorithm
import CoverageControlTorch as cct
import CoverageControlTorch.data_loaders.data_loader_utils as dl_utils
from CoverageControlTorch.utils.coverage_system import ToTensor
import CoverageControlTorch.utils.coverage_system as CoverageSystemUtils


class DatasetGenerator():
    def __init__(self, config_file):

        # Load configs and create directories
        self.config = dl_utils.LoadToml(config_file)
        self.data_dir = self.config['DataDir']
        self.data_folder = self.data_dir + '/data/'

        if not os.path.exists(self.data_dir):
            throw("Data directory does not exist")

        if not os.path.exists(self.data_folder):
            os.makedirs(self.data_folder)

        env_config_file = self.data_dir + self.config["EnvironmentConfig"]
        if not os.path.exists(env_config_file):
            throw("Environment config file does not exist")

        self.env_params = CoverageControl.Parameters(env_config_file)

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
        self.metrics_file = self.data_folder + 'metrics.txt'
        self.metrics = open(self.metrics_file, 'w')
        # Get current time
        start_time = datetime.datetime.now()
        self.metrics.write('Time: ' + str(datetime.datetime.now()) + '\n')
        self.metrics.write('Data directory: ' + self.data_dir + '\n')
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
        converged = not self.alg.Step()
        actions = self.alg.GetActions()
        count = self.dataset_count
        self.actions[count] = ToTensor(actions)
        self.robot_positions[count] = CoverageSystemUtils.GetRobotPositions(self.env)
        self.coverage_features[count] = CoverageSystemUtils.GetVoronoiFeatures(self.env)
        self.raw_local_maps[self.trigger_count] = CoverageSystemUtils.GetRawLocalMaps(self.env, self.env_params)
        self.raw_obstacle_maps[self.trigger_count] = CoverageSystemUtils.GetRawObstacleMaps(self.env, self.env_params)
        self.comm_maps[count] = CoverageSystemUtils.GetCommunicationMaps(self.env, self.env_params, self.cnn_map_size)
        self.edge_weights[count] = CoverageSystemUtils.GetWeights(self.env, self.env_params)
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
        resized_local_maps = CoverageSystemUtils.ResizeMaps(raw_local_maps, self.cnn_map_size)
        self.local_maps[self.trigger_start_idx:trigger_end_idx] = resized_local_maps.view(-1, self.num_robots, self.cnn_map_size, self.cnn_map_size).cpu().clone()

        raw_obstacle_maps = self.raw_obstacle_maps[0:trigger_end_idx - self.trigger_start_idx]
        raw_obstacle_maps = raw_obstacle_maps.to(self.device)
        resized_obstacle_maps = CoverageSystemUtils.ResizeMaps(raw_obstacle_maps, self.cnn_map_size)
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

        torch.save(train_tensor, self.data_folder + '/train/' + name)
        torch.save(validation_tensor, self.data_folder + '/val/' + name)
        torch.save(test_tensor, self.data_folder + '/test/' + name)

    def SaveDataset(self):
        as_sparse = self.config['SaveAsSparseQ']
        self.train_size = int(self.num_dataset * self.config['DatasetSplit']['TrainRatio'])
        self.validation_size = int(self.num_dataset * self.config['DatasetSplit']['ValRatio'])
        self.test_size = self.num_dataset - self.train_size - self.validation_size

        # Make sure the folder exists
        if not os.path.exists(self.data_folder + '/train'):
            os.makedirs(self.data_folder + '/train')
        if not os.path.exists(self.data_folder + '/val'):
            os.makedirs(self.data_folder + '/val')
        if not os.path.exists(self.data_folder + '/test'):
            os.makedirs(self.data_folder + '/test')

        self.SaveTensor(self.robot_positions, 'robot_positions.pt')
        self.SaveTensor(self.local_maps, 'local_maps.pt', as_sparse)
        self.SaveTensor(self.obstacle_maps, 'obstacle_maps.pt', as_sparse)
        self.SaveTensor(self.edge_weights, 'edge_weights.pt', as_sparse)

        # min_val, range_val = self.NormalizeCommunicationMaps()
        self.SaveTensor(self.comm_maps, 'comm_maps.pt', as_sparse)
        # torch.save(min_val, self.data_folder + '/comm_maps_min.pt')
        # torch.save(range_val, self.data_folder + '/comm_maps_range.pt')

        self.SaveTensor(self.actions, 'actions.pt')
        self.SaveTensor(self.coverage_features, 'coverage_features.pt')

        if self.config['NormalizeQ']:
            normalized_actions, actions_mean, actions_std = self.NormalizeTensor(self.actions)
            coverage_features, coverage_features_mean, coverage_features_std = self.NormalizeTensor(self.coverage_features)
            self.SaveTensor(normalized_actions, 'normalized_actions.pt')
            self.SaveTensor(coverage_features, 'normalized_coverage_features.pt')
            torch.save(actions_mean, self.data_folder + '/actions_mean.pt')
            torch.save(actions_std, self.data_folder + '/actions_std.pt')
            torch.save(coverage_features_mean, self.data_folder + '/coverage_features_mean.pt')
            torch.save(coverage_features_std, self.data_folder + '/coverage_features_std.pt')


    def StepWithoutSave(self):
        converged = not self.alg.Step()
        error_flag = self.env.StepActions(self.alg.GetActions())
        return converged or error_flag

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
    DatasetGenerator(config_file)


