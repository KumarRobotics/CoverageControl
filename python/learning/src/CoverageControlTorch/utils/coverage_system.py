import math
import numpy
import copy
import cv2
import torch
import torchvision
import torch_geometric
from torch_geometric.data import Dataset
import pyCoverageControl
import CoverageControlTorch as cct
from scipy.ndimage import gaussian_filter
from scipy.spatial import distance_matrix


def ToTensor(data):
    if isinstance(data, numpy.ndarray):
        return torch.from_numpy(numpy.copy(data.astype(numpy.float32)))
    elif isinstance(data, pyCoverageControl.PointVector):
        data_tensor = torch.Tensor(len(data), 2)
        for i in range(len(data)):
            data_tensor[i] = ToTensor(data[i])
        return data_tensor
    elif isinstance(data, pyCoverageControl.DblVectorVector):
        data_tensor = torch.Tensor(len(data))
        for i in range(len(data)):
            data_tensor[i] = ToTensor(data[i])
        return data_tensor
    elif isinstance(data, pyCoverageControl.DblVector):
        data_tensor = torch.Tensor(len(data))
        for i in range(len(data)):
            data_tensor[i] = float(data[i])
        return data_tensor
    else:
        raise ValueError('Unknown data type: {}'.format(type(data)))

def GetRawLocalMaps(env, params):
    local_maps = torch.zeros((env.GetNumRobots(), params.pLocalMapSize, params.pLocalMapSize))
    for r_idx in range(env.GetNumRobots()):
        local_maps[r_idx] = ToTensor(env.GetRobotLocalMap(r_idx))
    return local_maps

def GetRawObstacleMaps(env, params):
    obstacle_maps = torch.zeros((env.GetNumRobots(), params.pLocalMapSize, params.pLocalMapSize))
    for r_idx in range(env.GetNumRobots()):
        obstacle_maps[r_idx] = ToTensor(env.GetRobotObstacleMap(r_idx))
    return obstacle_maps

def GetCommunicationMaps(env, params, map_size):
    num_robots = env.GetNumRobots()
    # positions = env.GetRobotPositions()
    # robot_positions = ToTensor(env.GetRobotPositions())
    # relative_pos = robot_positions.unsqueeze(0) - robot_positions.unsqueeze(1)
    # scaled_relative_pos = torch.round(relative_pos * map_size / (params.pCommunicationRange * params.pResolution * 2.) + (map_size / 2. - params.pResolution / 2.))
    # relative_dist = relative_pos.norm(2, 2)
    # diagonal_mask = torch.eye(num_robots).to(torch.bool)
    # relative_dist.masked_fill_(diagonal_mask, params.pCommunicationRange + 1)

    comm_maps = torch.zeros((num_robots, 2, map_size, map_size))
    for r_idx in range(num_robots):
        neighbors_pos = ToTensor(env.GetRelativePositonsNeighbors(r_idx))
        scaled_indices = torch.round(neighbors_pos * map_size / (params.pCommunicationRange * params.pResolution * 2.) + (map_size / 2. - params.pResolution / 2.))
        # comm_range_mask = relative_dist[r_idx] < params.pCommunicationRange
        # scaled_indices = scaled_relative_pos[r_idx][comm_range_mask]
        indices = torch.transpose(scaled_indices, 1, 0)
        indices = indices.long()
        values = neighbors_pos / params.pCommunicationRange
        # values = values / params.pCommunicationRange
        # values = (values + params.pCommunicationRange) / (2. * params.pCommunicationRange)
        comm_maps[r_idx][0] = torch.sparse_coo_tensor(indices, values[:, 0], torch.Size([map_size, map_size])).to_dense()
        comm_maps[r_idx][1] = torch.sparse_coo_tensor(indices, values[:, 1], torch.Size([map_size, map_size])).to_dense()
    return comm_maps

def ResizeMaps(maps, resized_map_size):
    shape = maps.shape
    maps = maps.view(-1, maps.shape[-2], maps.shape[-1])
    maps = torchvision.transforms.functional.resize(maps, (resized_map_size, resized_map_size), interpolation=torchvision.transforms.InterpolationMode.BILINEAR, antialias=True)
    maps = maps.view(shape[:-2] + maps.shape[-2:])
    return maps

def GetMaps(env, params, resized_map_size, use_comm_map):

    num_robots = env.GetNumRobots()
    local_maps = torch.zeros((num_robots, params.pLocalMapSize, params.pLocalMapSize))
    obstacle_maps = torch.zeros((num_robots, params.pLocalMapSize, params.pLocalMapSize))

    for r_idx in range(num_robots):
        map = ToTensor(env.GetRobotLocalMap(r_idx))
        obstacle_map = ToTensor(env.GetRobotObstacleMap(r_idx))
        local_maps[r_idx] = map
        obstacle_maps[r_idx] = obstacle_map

    # Resize maps to resized_map_size
    local_maps = ResizeMaps(local_maps, resized_map_size)
    obstacle_maps = ResizeMaps(obstacle_maps, resized_map_size)
    # local_maps = torchvision.transforms.functional.resize(local_maps, (resized_map_size, resized_map_size), interpolation=torchvision.transforms.InterpolationMode.BILINEAR, antialias=True)
    # obstacle_maps = torchvision.transforms.functional.resize(obstacle_maps, (resized_map_size, resized_map_size), interpolation=torchvision.transforms.InterpolationMode.BILINEAR, antialias=True)
    if use_comm_map:
        comm_maps = GetCommunicationMaps(env, params, resized_map_size)
        maps = torch.cat([local_maps.unsqueeze(1), comm_maps, obstacle_maps.unsqueeze(1)], dim=1)
    else:
        maps = torch.cat([local_maps.unsqueeze(1), obstacle_maps.unsqueeze(1)], dim=1)
    return maps

def GetVoronoiFeatures(env):
    features = env.GetRobotVoronoiFeatures()
    tensor_features = torch.zeros((len(features), len(features[0])))
    for r_idx in range(len(features)):
        tensor_features[r_idx] = ToTensor(features[r_idx])
    return tensor_features

def GetRobotPositions(env):
    robot_positions = ToTensor(env.GetRobotPositions())
    return robot_positions

def GetWeights(env, params):
    onebyexp = 1. / math.exp(1.)
    robot_positions = ToTensor(env.GetRobotPositions())
    pairwise_distances = torch.cdist(robot_positions, robot_positions, 2)
    edge_weights = torch.exp(-(pairwise_distances.square())/(params.pCommunicationRange * params.pCommunicationRange))
    edge_weights.masked_fill_(edge_weights < onebyexp, 0)
    edge_weights.fill_diagonal_(0)
    return edge_weights

# Legacy edge weights used in previous research
# The weights are proportional to the distance
# Trying to move away from this
def RobotPositionsToEdgeWeights(robot_positions, world_map_size, comm_range):
    x = numpy.array(robot_positions)
    S = distance_matrix(x, x)
    S[S > comm_range] = 0
    C = (world_map_size**2) / (S.shape[0]**2)
    C = 3 / C
    graph_obs = C * S
    return graph_obs

def GetTorchGeometricData(env, params, use_cnn, use_comm_map, map_size):
    if use_cnn:
        features = GetMaps(env, params, map_size, use_comm_map)
    else:
        features = GetVoronoiFeatures(env)
    edge_weights = GetWeights(env, params).to_sparse()
    edge_index = edge_weights.indices().long()
    weights = edge_weights.values().float()
    data = torch_geometric.data.Data(x=features, edge_index=edge_index, edge_weight=weights)
    return data

# Legacy maps which gives decent results
# Trying to move away from this
def GetStableMaps(env, params, resized_map_size):
    robot_positions = ToTensor(env.GetRobotPositions())
    num_robots = env.GetNumRobots()
    maps = torch.empty((num_robots, 4, resized_map_size, resized_map_size))
    h_vals = torch.linspace(1.0, -1.0, maps.shape[-2]+1)
    h_vals = (h_vals[1:] + h_vals[:-1])/2
    w_vals = torch.linspace(-1.0, 1.0, maps.shape[-1]+1)
    w_vals = (w_vals[1:] + w_vals[:-1])/2
    heatmap_x = torch.stack([h_vals] * maps.shape[-1], dim=1)/100
    heatmap_y = torch.stack([w_vals] * maps.shape[-2], dim=0)/100
    for r_idx in range(num_robots):
        local_map = env.GetRobotLocalMap(r_idx)
        resized_local_map = cv2.resize(local_map, dsize=(resized_map_size, resized_map_size), interpolation=cv2.INTER_AREA)
        maps[r_idx][0] = torch.tensor(resized_local_map).float()

        comm_map = env.GetCommunicationMap(r_idx)
        filtered_comm_map = gaussian_filter(comm_map, sigma=(3,3), order=0)
        resized_comm_map = torch.tensor(cv2.resize(numpy.array(filtered_comm_map), dsize=(resized_map_size, resized_map_size), interpolation=cv2.INTER_AREA)).float()
        maps[r_idx][1] = resized_comm_map

        maps[r_idx][2] = heatmap_x
        maps[r_idx][3] = heatmap_y

    return maps
