import os
import numpy as np
import sys

import torch

import pyCoverageControl as cc# Main library
from pyCoverageControl import CoverageSystem
from pyCoverageControl import PointVector, Parameters, WorldIDF
from pyCoverageControl import OracleGlobalOffline, LloydLocalVoronoi, LloydGlobalOnline, LloydLocalSensorGlobalComm

import CoverageControlTorch as cct
from CoverageControlTorch.data_loaders import data_loader_utils as dl_utils
from CoverageControlTorch.data_loaders.data_loaders import LocalMapGNNDataset
# , GetStableMaps, RobotPositionsToEdgeWeights, ToTensor
import CoverageControlTorch.utils.coverage_system as CoverageSystemUtils

class Controller:
    def __init__(self, env, num_robots, map_size, model_file):
        self.num_robots = num_robots
        self.map_size = map_size
        self.model = torch.load(model_file)
        self.use_cnn = True
        self.use_comm_map = True
        if torch.cuda.is_available():
            self.device = torch.device('cuda')
        else:
            self.device = torch.device('cpu')

        self.actions_mean = self.model.actions_mean.to(self.device)
        self.actions_std = self.model.actions_std.to(self.device)
        self.model = self.model.to(self.device)
        self.model.eval()

    def Step(self, params, env):
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
        data = data.to(self.device)
        with torch.no_grad():
            actions = self.model(data)
        actions = actions * self.actions_std + self.actions_mean
        for i in range(self.num_robots):
            if actions[i][0] < 1e-3 and actions[i][1] < 1e-3:
                actions[i][0] = 0
                actions[i][1] = 0
        point_vector_actions = PointVector(actions.cpu().numpy())
        env.StepActions(point_vector_actions)
        return env.GetObjectiveValue(), False

class Evaluator:
    def __init__(self, env_file, pos_file, env_params_file, num_steps, cnn_map_size, model_file):
        self.env_config = dl_utils.LoadYaml(env_params_file)
        self.cc_params = cc.Parameters(env_params_file)

        self.num_robots = self.env_config['pNumRobots']
        self.num_features = self.env_config['pNumFeatures']
        self.num_steps = num_steps
        self.map_size = cnn_map_size
        self.model_file = model_file

    def Evaluate(self):

        cost_data = np.zeros(self.num_steps)
        if os.path.isfile(env_file) and os.path.isfile(pos_file):
            world_idf = WorldIDF(self.cc_params, env_file)
            env_main = CoverageSystem(self.cc_params, world_idf, pos_file)
        else:
            print("New environment")
            env_main = CoverageSystem(self.cc_params, self.num_features, self.num_robots)
            world_idf = env_main.GetWorldIDFObject()

        robot_init_pos = env_main.GetRobotPositions()
        env = CoverageSystem(self.cc_params, world_idf, robot_init_pos)
        controller = Controller(env, self.num_robots, self.map_size, self.model_file)
        step_count = 0
        cost_data[step_count] = env.GetObjectiveValue()
        print("Initial objective value: ", cost_data[step_count])
        step_count = step_count + 1
        while step_count < self.num_steps:
            objective_value, converged = controller.Step(self.cc_params, env)
            cost_data[step_count] = objective_value
            if converged:
                cost_data[step_count:] = objective_value
                break
            step_count = step_count + 1
            if step_count % 100 == 0:
                print(f"Step: {step_count}")

        print(f"Objective value: {cost_data[-1]}")
        return cost_data


if __name__ == "__main__":


    env_file = sys.argv[1]
    pos_file = sys.argv[2]
    env_params_file = sys.argv[3]
    num_steps = int(sys.argv[4])
    cnn_map_size = int(sys.argv[5])
    model_file = sys.argv[6]
    evaluator = Evaluator(env_file, pos_file, env_params_file, num_steps, cnn_map_size, model_file)
    evaluator.Evaluate()
