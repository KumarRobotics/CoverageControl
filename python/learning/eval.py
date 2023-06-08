import os
import numpy as np
import sys

import torch

import pyCoverageControl as cc# Main library
from pyCoverageControl import CoverageSystem
from pyCoverageControl import PointVector, Parameters, WorldIDF
from pyCoverageControl import OracleGlobalOffline, LloydLocalVoronoi, LloydGlobalOnline, LloydLocalSensorGlobalComm

from torch_geometric.data import Data
import torchvision.transforms as T

import CoverageControlTorch as cct
from CoverageControlTorch.data_loaders import data_loader_utils as dl_utils
from CoverageControlTorch.data_loaders.data_loaders import LocalMapGNNDataset

class Controller:
    def __init__(self, config, params, env, num_robots, map_size):
        self.config = config
        self.params = params
        self.env = env
        self.num_robots = num_robots
        self.name = self.config['Name']
        self.type = self.config['Type']
        self.map_size = map_size
        self.resizer = Resizer(self.map_size)
        if self.type == 'Learning':
            self.Step = self.StepLearning
            self.model_file = self.config['ModelFile']
            self.model = torch.load(self.model_file)
            self.use_cnn = self.config['UseCNN']
            self.use_comm_map = self.config['UseCommMap']
            self.model_dict = torch.load(self.model_file)
            if torch.cuda.is_available():
                self.device = torch.device('cuda')
            else:
                self.device = torch.device('cpu')
            self.model = self.model.to(self.device)
            self.model.eval()
        elif self.type == 'CoverageControl':
            self.Step = self.StepCoverageControl
            if self.name == 'OracleGlobalOffline':
                self.cc = OracleGlobalOffline(self.params, self.num_robots, self.env)
            elif self.name == 'LloydLocalVoronoi':
                self.cc = LloydLocalVoronoi(self.params, self.num_robots, self.env)
            elif self.name == 'LloydGlobalOnline':
                self.cc = LloydGlobalOnline(self.params, self.num_robots, self.env)
            elif self.name == 'LloydLocalSensorGlobalComm':
                self.cc = LloydLocalSensorGlobalComm(self.params, self.num_robots, self.env)
            else:
                raise ValueError('Unknown controller type: {}'.format(self.type))

    def StepCoverageControl(self):
        self.cc.Step()
        actions = self.cc.GetActions()
        self.env.StepActions(actions)
        return self.env.GetObjectiveValue()

    def StepLearning(self):
        if self.use_cnn:
            features = self.GetMaps(self.use_comm_map)
        else:
            features = self.env.GetLocalVoronoiFeaturesTensor()
        edge_weights = self.env.GetEdgeWeights()
        data = dl_utils.ToTorchGeometricData(features, edge_weights)
        data = data.to(self.device)
        with torch.no_grad():
            actions = self.model(data)
        point_vector_actions = PointVector(actions.cpu().numpy())
        self.env.StepActions(point_vector_actions)
        return self.env.GetObjectiveValue()

    def GetMaps(self, use_comm_map):

        local_maps = torch.tensor(self.env.GetRobotLocalMap(0)).clone()
        obstacle_maps = torch.tensor(self.env.GetRobotObstacleMap(0)).clone()
        local_maps = local_maps.unsqueeze(0)
        obstacle_maps = obstacle_maps.unsqueeze(0)
        for i in range(1, self.num_robots):
            local_maps = torch.cat([local_maps, torch.tensor(self.env.GetRobotLocalMap(i)).unsqueeze(0)], 0)
            obstacle_maps = torch.cat([obstacle_maps, torch.tensor(self.env.GetRobotObstacleMap(i)).unsqueeze(0)], 0)

        self.resizer.to(self.device)
        local_maps = local_maps.to(self.device)
        local_maps_resized = self.resizer.forward(local_maps)
        local_maps_resized = local_maps_resized.unsqueeze(1).to('cpu')

        obstacle_maps = obstacle_maps.to(self.device)
        obstacle_maps_resized = self.resizer.forward(obstacle_maps)
        obstacle_maps_resized = obstacle_maps_resized.unsqueeze(1).to('cpu')

        if use_comm_map == True:
            communication_maps = torch.Tensor((self.num_robots, self.num_robots, self.map_size, self.map_size))
            communication_maps = self.env.GetAllRobotsCommunicationMaps(self.map_size)
            communication_maps.to_dense()
            maps = torch.cat([local_maps, comm_maps, obstacle_maps], 1)
        else:
            maps = torch.cat([local_maps, obstacle_maps], 1)
        return maps

class Resizer(torch.nn.Module):
    def __init__(self, size):
        super(Resizer, self).__init__()
        self.size = size
        self.T = T.Resize(size, interpolation=T.InterpolationMode.BILINEAR, antialias=True)

    def forward(self, img):
        return self.T(img)

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
        self.env_config = dl_utils.LoadYaml(self.env_config_file)
        self.cc_params = cc.Parameters(self.env_config_file)

        self.num_robots = self.env_config['pNumRobots']
        self.num_features = self.env_config['pNumFeatures']
        self.num_envs = self.config['NumEnvironments']
        self.num_steps = self.config['NumSteps']
        self.map_size = self.config['MapSize']

    def Evaluate(self):
        dataset_count = 0

        cost_data = np.zeros((self.num_controllers, self.num_envs, self.num_steps))
        while dataset_count < self.num_envs:
            print("New environment")

            if dataset_count > 0:
                del world_idf
                del env_main
            pos_file = self.env_path + str(dataset_count) + ".pos"
            env_file = self.env_path + str(dataset_count) + ".env"
            if os.path.isfile(env_file) and os.path.isfile(pos_file):
                world_idf = WorldIDF(self.cc_params, env_file)
                env_main = CoverageSystem(self.cc_params, world_idf, pos_file)
            else:
                env_main = CoverageSystem(self.cc_params, self.num_features, self.num_robots)
                env_main.WriteEnvironment(pos_file, env_file)
                world_idf = env_main.GetWorldIDFObject()

            robot_init_pos = env_main.GetRobotPositions()
            for controller_id in range(self.num_controllers):
                step_count = 0
                env = CoverageSystem(self.cc_params, world_idf, robot_init_pos)
                controller = Controller(self.controllers[controller_id], self.cc_params, env, self.num_robots, self.map_size)

                while step_count < self.num_steps:
                    cost_data[controller_id, dataset_count, step_count] = controller.Step()
                    step_count = step_count + 1
                    if step_count % 100 == 0:
                        print(f"Environment {dataset_count}, Controller {controller_id}, Step {step_count}")

                del controller
                # Save controller data
                controller_dir = self.eval_dir + '/' + self.controllers[controller_id]['Name']
                controller_data_file = controller_dir + '/' + 'eval.csv'
                np.savetxt(controller_data_file, cost_data[controller_id, :dataset_count, :], delimiter=",")


            dataset_count = dataset_count + 1


if __name__ == "__main__":

    config_file = sys.argv[1]
    config = dl_utils.LoadYaml(config_file)

    evaluator = Evaluator(config)
    evaluator.Evaluate()
