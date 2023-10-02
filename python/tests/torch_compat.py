import numpy as np
import math
import pyCoverageControl # Main library
from pyCoverageControl import Point2 # for defining points
from pyCoverageControl import PointVector # for defining list of points
from pyCoverageControl import BivariateNormalDistribution as BND # for defining bivariate normal distributions
from pyCoverageControl import CoverageSystem # for defining coverage system

import CoverageControlTorch as cct
from CoverageControlTorch.data_loaders import data_loader_utils as dl_utils
from CoverageControlTorch.data_loaders.data_loaders import LocalMapGNNDataset
from CoverageControlTorch.utils.coverage_system import GetTorchGeometricData
# , GetStableMaps, RobotPositionsToEdgeWeights, ToTensor
import CoverageControlTorch.utils.coverage_system as CoverageSystemUtils

params_ = pyCoverageControl.Parameters()
params_.pNumRobots = 10

robot_positions = PointVector()
robot_positions.append(np.array([10, 10]))
robot_positions.append(robot_positions[0] - Point2(-60, 50));
robot_positions.append(robot_positions[0] + Point2(-50, 60));
robot_positions.append(robot_positions[0] - Point2(50, 50));
robot_positions.append(robot_positions[0] + Point2(60, 60));
robot_positions.append(robot_positions[0] - Point2(0, 50));
robot_positions.append(robot_positions[0] + Point2(0, 60));
robot_positions.append(robot_positions[0] - Point2(50, 0));
robot_positions.append(robot_positions[0] + Point2(60, 0));
robot_positions.append(robot_positions[0] + Point2(500, 0));


from pyCoverageControl import WorldIDF # for defining world idf
world_idf = WorldIDF(params_)

mean = Point2(500, 500)
sigma = 10
peak_val = 1
dist = BND(mean, sigma, peak_val) # circular gaussian

mean = Point2(300, 500)
dist1 = BND(mean, sigma, peak_val) # circular gaussian

world_idf.AddNormalDistribution(dist); # Add a distribution to the idf
world_idf.AddNormalDistribution(dist1); # Add a distribution to the idf
env = CoverageSystem(params_, world_idf, robot_positions)

comm_maps = CoverageSystemUtils.GetCommunicationMaps(env, params_, 32)
import torch
cmap = torch.jit.load("comm_map.pt")
comm_maps1 = cmap.state_dict()['0']

# Check if comm_maps and comm_maps1 are the same
print(torch.allclose(comm_maps, comm_maps1))

world_map = CoverageSystemUtils.ToTensor(env.GetWorldIDF())
world_map1 = torch.jit.load("world_map.pt")
world_map1 = world_map1.state_dict()['0']

# Check if world_map and world_map1 are the same
print(torch.allclose(world_map, world_map1))

comm_maps_resized = CoverageSystemUtils.ResizeMaps(comm_maps, 32)
cmap = torch.jit.load("comm_map_resized.pt")
comm_maps_resized1 = cmap.state_dict()['0']

# Check if comm_maps_resized and comm_maps_resized1 are the same
print(torch.allclose(comm_maps_resized, comm_maps_resized1))

# Print number of non-zero elements in comm_maps
print(torch.count_nonzero(comm_maps))
print(torch.count_nonzero(comm_maps1))
