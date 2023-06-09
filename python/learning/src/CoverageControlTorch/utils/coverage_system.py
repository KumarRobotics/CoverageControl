import math
import numpy
import torch
import torchvision
import torch_geometric
from torch_geometric.data import Dataset
import pyCoverageControl
import CoverageControlTorch as cct


def ToTensor(data):
    if isinstance(data, numpy.ndarray):
        return torch.from_numpy(numpy.copy(data.astype(numpy.float32)))
    elif isinstance(data, pyCoverageControl.PointVector):
        data_tensor = torch.Tensor(len(data), 2)
        for i in range(len(data)):
            data_tensor[i] = ToTensor(data[i])
        return data_tensor
    elif isinstance(data, torch.Tensor):
        return data.float()
    else:
        raise ValueError('Unknown data type: {}'.format(type(data)))

def GetCommunicationMaps(env, params, map_size):
    num_robots = env.GetNumRobots()
    positions = env.GetRobotPositions()
    robot_positions = ToTensor(env.GetRobotPositions())
    comm_maps = torch.Tensor((num_robots, 2, map_size, map_size))
    relative_pos = robot_positions.unsqueeze(0) - robot_positions.unsqueeze(1)
    scaled_relative_pos = torch.round(relative_pos * map_size / (params.pCommunicationRange * params.pResolution * 2.) + (map_size / 2. - params.pResolution / 2.))
    relative_dist = relative_pos.norm(2, 2)
    diagonal_mask = torch.eye(num_robots).to(torch.bool)
    relative_dist.masked_fill_(diagonal_mask, params.pCommunicationRange + 1)

    comm_maps = torch.zeros((num_robots, 2, map_size, map_size))
    for r_idx in range(num_robots):
        comm_range_mask = relative_dist[r_idx] < params.pCommunicationRange
        scaled_indices = scaled_relative_pos[r_idx][comm_range_mask]
        indices = torch.transpose(scaled_indices, 1, 0)
        indices = indices.long()
        values = relative_pos[r_idx][comm_range_mask]/params.pCommunicationRange
        comm_maps[r_idx][0] = torch.sparse.FloatTensor(indices, values[:, 0], torch.Size([map_size, map_size])).to_dense()
        comm_maps[r_idx][1] = torch.sparse.FloatTensor(indices, values[:, 1], torch.Size([map_size, map_size])).to_dense()
        return comm_maps

def GetMaps(env, params, map_size, use_comm_map):

    num_robots = env.GetNumRobots()
    local_maps = torch.zeros((num_robots, params.pLocalMapSize, params.pLocalMapSize))
    obstacle_maps = torch.zeros((num_robots, params.pLocalMapSize, params.pLocalMapSize))

    for r_idx in range(num_robots):
        map = ToTensor(env.GetRobotLocalMap(r_idx))
        obstacle_map = ToTensor(env.GetRobotObstacleMap(r_idx))
        local_maps[r_idx] = map
        obstacle_maps[r_idx] = obstacle_map

    # Resize maps to map_size
    local_maps = torchvision.transforms.functional.resize(local_maps, (map_size, map_size), interpolation=torchvision.transforms.InterpolationMode.BILINEAR, antialias=True)
    obstacle_maps = torchvision.transforms.functional.resize(obstacle_maps, (map_size, map_size), interpolation=torchvision.transforms.InterpolationMode.BILINEAR, antialias=True)
    if use_comm_map:
        comm_maps = GetCommunicationMaps(env, params, map_size)
        maps = torch.cat([local_maps.unsqueeze(1), comm_maps, obstacle_maps.unsqueeze(1)], dim=1)
    else:
        maps = torch.cat([local_maps.unsqueeze(1), obstacle_maps.unsqueeze(1)], dim=1)
    return maps

def GetVoronoiFeatures(env, params):
    features = ToTensor(env.GetVoronoiFeatures())
    return features

def GetWeights(env, params):
    onebyexp = 1. / math.exp(1.)
    robot_positions = ToTensor(env.GetRobotPositions())
    pairwise_distances = torch.cdist(robot_positions, robot_positions, 2)
    edge_weights = torch.exp(-(pairwise_distances.square())/(params.pCommunicationRange * params.pCommunicationRange))
    edge_weights.masked_fill_(edge_weights < onebyexp, 0)
    edge_weights.fill_diagonal_(0)
    return edge_weights

def GetTorchGeometricData(env, params, use_cnn, use_comm_map, map_size):
    if use_cnn:
        features = GetMaps(env, params, map_size, use_comm_map)
    else:
        features = GetVoronoiFeatures(env, params)
    edge_weights = GetWeights(env, params).to_sparse()
    edge_index = edge_weights.indices().long()
    weights = edge_weights.values().float()
    data = torch_geometric.data.Data(x=features, edge_index=edge_index, edge_weight=weights)
    return data

def ToTorchGeometricDataRobotPositions(env, params, use_cnn, use_comm_map, map_size):
    if use_cnn:
        features = GetMaps(env, params, map_size, use_comm_map)
    else:
        features = GetVoronoiFeatures(env, params)
    x = ToTensor(env.GetRobotPositions())
    dist_matrix = torch.cdist(x, x, 2)
    dist_matrix[dist_matrix > params.pCommunicationRange] = 0
    C = (params.pWorldMapSize **2)/(dist_matrix.shape[0] ** 2)
    C = 3/C
    edge_weights = dist_matrix * C
    edge_weights = edge_weights.to_sparse()
    edge_index = edge_weights.indices().long()
    weights = edge_weights.values().float()
    data = torch_geometric.data.Data(
            x=feature,
            edge_index=edge_index,
            edge_weight=weights,
            )
    return data
