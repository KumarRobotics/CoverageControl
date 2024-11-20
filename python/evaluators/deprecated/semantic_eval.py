import os
import csv
import numpy as np
import sys

import torch
from numpy import genfromtxt

import pyCoverageControl as cc# Main library
from pyCoverageControl import CoverageSystem
from pyCoverageControl import PointVector, Parameters, WorldIDF
from pyCoverageControl import OracleGlobalOffline, LloydLocalVoronoi, LloydGlobalOnline, LloydLocalSensorGlobalComm

import CoverageControlTorch as cct
from CoverageControlTorch.data_loaders import data_loader_utils as dl_utils
from CoverageControlTorch.data_loaders.data_loaders import LocalMapGNNDataset
from CoverageControlTorch.utils.coverage_system import GetTorchGeometricData
# , GetStableMaps, RobotPositionsToEdgeWeights, ToTensor
import CoverageControlTorch.utils.coverage_system as CoverageSystemUtils

class Controller:
    def __init__(self, config, params, env, num_robots, map_size):
        self.config = config
        self.num_robots = num_robots
        self.name = self.config['Name']
        self.type = self.config['Type']
        self.map_size = map_size
        if self.type == 'Learning':
            self.Step = self.StepLearning
            self.model_file = self.config['ModelFile']
            self.model = torch.load(self.model_file)
            self.use_cnn = self.config['UseCNN']
            self.use_comm_map = self.config['UseCommMap']
            if torch.cuda.is_available():
                self.device = torch.device('cuda')
            else:
                self.device = torch.device('cpu')
            # Get actions_mean and actions_std from model buffer
            self.actions_mean = self.model.actions_mean.to(self.device)
            self.actions_std = self.model.actions_std.to(self.device)
            self.model = self.model.to(self.device)
            self.model.eval()
        elif self.type == 'CoverageControl':
            self.Step = self.StepCoverageControl
            if self.name == 'OracleGlobalOffline':
                self.cc = OracleGlobalOffline(params, self.num_robots, env)
            elif self.name == 'LloydLocalVoronoi':
                self.cc = LloydLocalVoronoi(params, self.num_robots, env)
            elif self.name == 'LloydGlobalOnline':
                self.cc = LloydGlobalOnline(params, self.num_robots, env)
            elif self.name == 'LloydLocalSensorGlobalComm':
                self.cc = LloydLocalSensorGlobalComm(params, self.num_robots, env)
            else:
                raise ValueError('Unknown controller type: {}'.format(self.type))

    def StepCoverageControl(self, params, env):
        continue_flag = self.cc.Step()
        converged = not continue_flag
        actions = self.cc.GetActions()
        env.StepActions(actions)
        objective_value = env.GetObjectiveValue()
        return env.GetObjectiveValue(), converged

    def StepLearning(self, params, env):
        # maps = GetStableMaps(env, params, self.map_size)
        # edge_weights = RobotPositionsToEdgeWeights(robot_positions, params.pWorldMapSize, params.pCommunicationRange)
        # edge_weights = RobotPositionsToEdgeWeights(env.GetRobotPositions(), params.pWorldMapSize, params.pCommunicationRange)
        raw_local_maps = CoverageSystemUtils.GetRawLocalMaps(env, params).to(self.device)
        resized_local_maps = CoverageSystemUtils.ResizeMaps(raw_local_maps, self.map_size)
        raw_obstable_maps = CoverageSystemUtils.GetRawObstacleMaps(env, params).to(self.device)
        resized_obstacle_maps = CoverageSystemUtils.ResizeMaps(raw_obstable_maps, self.map_size)
        comm_maps = CoverageSystemUtils.GetCommunicationMaps(env, params, self.map_size).to(self.device)
        edge_weights = CoverageSystemUtils.GetWeights(env, params)
        # maps = torch.cat([resized_local_maps.unsqueeze(1), resized_obstacle_maps.unsqueeze(1)], 1)
        maps = torch.cat([resized_local_maps.unsqueeze(1), comm_maps, resized_obstacle_maps.unsqueeze(1)], 1)

        robot_positions = CoverageSystemUtils.GetRobotPositions(env)
        robot_positions = (robot_positions + params.pWorldMapSize/2) / params.pWorldMapSize

        data = dl_utils.ToTorchGeometricData(maps, edge_weights, robot_positions)
        # data = GetTorchGeometricData(env, params, self.use_cnn, self.use_comm_map, self.map_size)
        data = data.to(self.device)
        with torch.no_grad():
            actions = self.model(data)
        actions = actions * self.actions_std + self.actions_mean
        point_vector_actions = PointVector(actions.cpu().numpy())
        env.StepActions(point_vector_actions)
        return env.GetObjectiveValue(), False

class Evaluator:
    def __init__(self, config):
        self.config = config
        self.eval_dir = self.config['EvalDir']
        self.env_path = self.eval_dir + '/envs/'

        with open(self.env_path + '/cities_features.dat', 'r') as file:
            csv_reader = csv.reader(file, delimiter=',')
            self.city_names = []
            for row in csv_reader:
                self.city_names.append(row[0])

        self.controllers = self.config['Controllers']
        self.num_controllers = len(self.controllers)
        for controller in self.controllers:
            controller_dir = self.eval_dir + '/' + controller['Name']
            if not os.path.exists(controller_dir):
                os.makedirs(controller_dir)

        self.env_config_file = self.config['EnvironmentConfig']
        self.env_config = dl_utils.LoadToml(self.env_config_file)
        self.cc_params = cc.Parameters(self.env_config_file)

        self.num_robots = self.env_config['pNumRobots']
        self.num_features = self.env_config['pNumFeatures']
        self.world_map_size = self.env_config['EnvMaps']['pWorldMapSize']
        self.num_envs = self.config['NumEnvironments']
        self.num_steps = self.config['NumSteps']
        self.map_size = self.config['MapSize']

    def Evaluate(self, save = True):

        num_trials = 10
        cost_data = np.zeros((self.num_controllers, self.num_envs, num_trials, self.num_steps))
        # for controller_id in range(self.num_controllers):
        #     controller_dir = self.eval_dir + '/' + self.controllers[controller_id]['Name']
        #     controller_data_file = controller_dir + '/' + 'eval.csv'
        #     controller_data = genfromtxt(controller_data_file, delimiter=',')
        #     cost_data[controller_id, :, :] = controller_data

        dataset_count = 0
        while dataset_count < self.num_envs:
            print("New environment")

            env_name = self.city_names[dataset_count]

            idf_file = self.env_path + '/' + env_name + '/idf.dat'
            world_idf = WorldIDF(self.cc_params, idf_file)

            for trial_count in range(num_trials):
                # Load positions from file
                # pos_file = self.env_path + '/' + env_name + '/pos.dat'
                # env_main = CoverageSystem(self.cc_params, world_idf, pos_file)
                # robot_init_pos = env_main.GetRobotPositions()

                # Generate random positions
                robot_init_pos = PointVector()
                for robot_id in range(self.num_robots):
                    robot_init_pos.append(np.array([np.random.uniform(0, self.world_map_size), np.random.uniform(0, self.world_map_size)]))

                # env_main = CoverageSystem(self.cc_params, world_idf, robot_init_pos)

                # env_main.PlotInitMap(self.env_path + '/' + env_name, "InitMap")

                for controller_id in range(self.num_controllers):
                    step_count = 0
                    env = CoverageSystem(self.cc_params, world_idf, robot_init_pos)

                    # map_dir = self.eval_dir + '/' + self.controllers[controller_id]['Name'] + '/plots/'
                    # os.makedirs(map_dir, exist_ok = True)
                    # env.PlotInitMap(map_dir, "InitMap")
                    # env.RecordPlotData()
                    # env.PlotMapVoronoi(map_dir, step_count)

                    controller = Controller(self.controllers[controller_id], self.cc_params, env, self.num_robots, self.map_size)
                    cost_data[controller_id, dataset_count, trial_count, step_count] = env.GetObjectiveValue()
                    step_count = step_count + 1
                    while step_count < self.num_steps:
                        objective_value, converged = controller.Step(self.cc_params, env)
                        cost_data[controller_id, dataset_count, trial_count, step_count] = objective_value
                        if converged:
                            cost_data[controller_id, dataset_count, trial_count, step_count:] = objective_value
                            break
                        # env.PlotMapVoronoi(map_dir, step_count)
                        # env.RecordPlotData()
                        step_count = step_count + 1
                        if step_count % 100 == 0:
                            print(f"Environment {dataset_count}, Controller {controller_id}, Trial {trial_count}, Step {step_count}")

                    if save == True:
                        controller_dir = self.eval_dir + '/' + self.controllers[controller_id]['Name']
                        controller_data_file = controller_dir + '/' + 'eval.csv'
                        # np.savetxt(controller_data_file, cost_data[controller_id, :dataset_count + 1, :], delimiter=",")
                        # np.savetxt(controller_data_file, cost_data[controller_id, :, :], delimiter=",")
                        # change (controllers, envs, trials, steps) to (controllers, envs * trials, steps)
                        np.savetxt(controller_data_file, cost_data[controller_id, :, :, :].reshape(self.num_envs * num_trials, self.num_steps), delimiter=",")
                    # env.RenderRecordedMap(self.eval_dir + '/' + self.controllers[controller_id]['Name'] + '/', 'video.mp4')
                    del controller
                    del env
            dataset_count = dataset_count + 1
        return cost_data


if __name__ == "__main__":

    config_file = sys.argv[1]
    config = dl_utils.LoadToml(config_file)

    evaluator = Evaluator(config)
    evaluator.Evaluate()
