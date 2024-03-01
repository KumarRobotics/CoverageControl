import os
import numpy as np
import sys

import torch

import CoverageControl as cc# Main library
from CoverageControl import CoverageSystem
from CoverageControl import PointVector, Parameters, WorldIDF
from CoverageControl import OracleGlobalOffline, LloydLocalVoronoi, LloydGlobalOnline, LloydLocalSensorGlobalComm

import CoverageControlTorch as cct
from CoverageControlTorch.data_loaders import data_loader_utils as dl_utils
from CoverageControlTorch.data_loaders.data_loaders import LocalMapGNNDataset
from CoverageControlTorch.utils.coverage_system import GetTorchGeometricData
# , GetStableMaps, RobotPositionsToEdgeWeights, ToTensor
import CoverageControlTorch.utils.coverage_system as CoverageSystemUtils
from CoverageControlTorch.models.gnn import CNNGNN

class Controller:
    def __init__(self, config, params, env, num_robots, map_size):
        self.config = config
        self.num_robots = num_robots
        self.name = self.config['Name']
        self.type = self.config['Type']
        self.map_size = map_size
        if self.type == 'Learning':
            if torch.cuda.is_available():
                self.device = torch.device('cuda')
            else:
                self.device = torch.device('cpu')
            self.Step = self.StepLearning
            # Check if ModelFile is provided
            if 'ModelFile' in self.config:
                self.model_file = self.config['ModelFile']
                self.model = torch.load(self.model_file)
            else: # Load from ModelStateDict
                self.learning_config_file = self.config['LearningConfig']
                self.learning_config = dl_utils.LoadToml(self.learning_config_file)
                self.model = CNNGNN(self.learning_config).to(self.device)
                self.model.LoadModel(self.config['ModelStateDict'])
            self.use_cnn = self.config['UseCNN']
            self.use_comm_map = self.config['UseCommMap']
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
        return env.GetObjectiveValue(), converged

    def StepLearning(self, params, env):
        # maps = GetStableMaps(env, params, self.map_size)
        # edge_weights = RobotPositionsToEdgeWeights(robot_positions, params.pWorldMapSize, params.pCommunicationRange)
        # edge_weights = RobotPositionsToEdgeWeights(env.GetRobotPositions(), params.pWorldMapSize, params.pCommunicationRange)
        raw_local_maps = CoverageSystemUtils.GetRawLocalMaps(env, params).to(self.device)
        resized_local_maps = CoverageSystemUtils.ResizeMaps(raw_local_maps, self.map_size)
        raw_obstable_maps = CoverageSystemUtils.GetRawObstacleMaps(env, params).to(self.device)
        resized_obstacle_maps = CoverageSystemUtils.ResizeMaps(raw_obstable_maps, self.map_size)
        edge_weights = CoverageSystemUtils.GetWeights(env, params)
        if self.use_comm_map:
            comm_maps = CoverageSystemUtils.GetCommunicationMaps(env, params, self.map_size).to(self.device)
            maps = torch.cat([resized_local_maps.unsqueeze(1), comm_maps, resized_obstacle_maps.unsqueeze(1)], 1)
        else:
            maps = torch.cat([resized_local_maps.unsqueeze(1), resized_obstacle_maps.unsqueeze(1)], 1)

        robot_positions = CoverageSystemUtils.GetRobotPositions(env)
        robot_positions = (robot_positions + params.pWorldMapSize/2) / params.pWorldMapSize

        data = dl_utils.ToTorchGeometricData(maps, edge_weights, robot_positions)
        # data = GetTorchGeometricData(env, params, self.use_cnn, self.use_comm_map, self.map_size)
        data = data.to(self.device)
        with torch.no_grad():
            actions = self.model(data)
        actions = actions * self.actions_std + self.actions_mean
        # for i in range(self.num_robots):
        #     if actions[i][0] < 1e-3 and actions[i][1] < 1e-3:
        #         actions[i][0] = 0
        #         actions[i][1] = 0
        point_vector_actions = PointVector(actions.cpu().numpy())
        env.StepActions(point_vector_actions)
        return env.GetObjectiveValue(), False

class Evaluator:
    def __init__(self, config):
        self.config = config
        self.eval_dir = self.config['EvalDir']
        self.env_path = self.eval_dir + '/envs/'
        if not os.path.exists(self.env_path):
            os.makedirs(self.env_path)

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
        self.num_envs = self.config['NumEnvironments']
        self.num_steps = self.config['NumSteps']
        self.map_size = self.config['MapSize']

    def Evaluate(self, save = True):
        dataset_count = 0

        while dataset_count < self.num_envs:
            dataset_count = 74
            print(f"Environment {dataset_count}")
            pos_file = self.env_path + str(dataset_count) + ".pos"
            env_file = self.env_path + str(dataset_count) + ".env"
            if os.path.isfile(env_file) and os.path.isfile(pos_file):
                world_idf = WorldIDF(self.cc_params, env_file)
                env_main = CoverageSystem(self.cc_params, world_idf, pos_file)
            else:
                print("New environment")
                env_main = CoverageSystem(self.cc_params, self.num_features, self.num_robots)
                env_main.WriteEnvironment(pos_file, env_file)
                world_idf = env_main.GetWorldIDFObject()

            robot_init_pos = env_main.GetRobotPositions(force_no_noise = True)
            for controller_id in range(self.num_controllers):
                objective_values = np.zeros(self.num_steps)
                print(f"Controller {controller_id}")
                step_count = 0
                env = CoverageSystem(self.cc_params, world_idf, robot_init_pos)

                map_dir = self.eval_dir + '/' + self.controllers[controller_id]['Name'] + '/plots/'
                os.makedirs(map_dir, exist_ok = True)
                env.PlotInitMap(map_dir, "InitMap")
                env.RecordPlotData()
                # env.PlotMapVoronoi(map_dir, step_count)

                controller = Controller(self.controllers[controller_id], self.cc_params, env, self.num_robots, self.map_size)
                objective_value = env.GetObjectiveValue()
                initial_objective_value = objective_value
                objective_values[step_count] = 1
                step_count = step_count + 1
                while step_count < self.num_steps:
                    objective_value, converged = controller.Step(self.cc_params, env)
                    objective_values[step_count] = objective_value/initial_objective_value
                    # env.PlotMapVoronoi(map_dir, step_count)
                    env.RecordPlotData()
                    step_count = step_count + 1
                    if step_count % 100 == 0:
                        print(f"Step {step_count}, Objective Value {objective_value}")
                        print(f"Environment {dataset_count}, Controller {controller_id}, Step {step_count}")
                    # if converged:
                    #     break

                if save == True:
                    controller_dir = self.eval_dir + '/' + self.controllers[controller_id]['Name']
                # Write objective values to file
                objective_values_file = self.eval_dir + '/' + self.controllers[controller_id]['Name'] + '/objective_values_' + str(dataset_count) + '.csv'
                np.savetxt(objective_values_file, objective_values, delimiter = ',')

                env.RenderRecordedMap(self.eval_dir + '/' + self.controllers[controller_id]['Name'] + '/', 'video.mp4')
                del controller
                del env
            dataset_count = dataset_count + 1


if __name__ == "__main__":

    config_file = sys.argv[1]
    config = dl_utils.LoadToml(config_file)

    evaluator = Evaluator(config)
    evaluator.Evaluate()
