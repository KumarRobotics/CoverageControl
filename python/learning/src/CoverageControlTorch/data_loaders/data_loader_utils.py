import yaml
import os
import torch
import torch_geometric

"""
Function to load a tensor from a file
Checks if the file exists, if not, returns None
Checks if the loaded data is a tensor or is in jit script format
"""
def LoadTensor(path):
    # Throw error if path does not exist
    if not os.path.exists(path):
        raise FileNotFoundError(f"data_loader_utils::LoadTensor: File not found: {path}")
    # Load data
    data = torch.load(path)
    # Extract tensor if data is in jit script format
    if isinstance(data, torch.jit.ScriptModule):
        tensor = list(data.parameters())[0]
    else:
        tensor = data
    return tensor

def LoadYaml(path):
    # Throw error if path does not exist
    if not os.path.exists(path):
        raise FileNotFoundError(f"data_loader_utils::LoadYaml: File not found: {path}")
    # Load data
    with open(path, "r") as f:
        data = yaml.load(f, Loader=yaml.FullLoader)
    return data

def LoadMaps(path, use_comm_map):
    local_maps = LoadTensor(f"{path}/local_maps.pt")
    local_maps = local_maps.unsqueeze(2)
    obstacle_maps = LoadTensor(f"{path}/obstacle_maps.pt")
    obstacle_maps = obstacle_maps.to_dense().unsqueeze(2)

    if use_comm_map:
        comm_maps = LoadTensor(f"{path}/comm_maps.pt")
        comm_maps = comm_maps.to_dense()
        maps = torch.cat([local_maps, comm_maps, obstacle_maps], 2)
    else:
        maps = torch.cat([local_maps, obstacle_maps], 2)
    return maps

def LoadFeatures(path, output_dim = None):
    normalized_coverage_features = LoadTensor(f"{path}/normalized_coverage_features.pt")
    coverage_features_mean = LoadTensor(f"{path}/../coverage_features_mean.pt")
    coverage_features_std = LoadTensor(f"{path}/../coverage_features_std.pt")
    if output_dim is not None:
        normalized_coverage_features = normalized_coverage_features[:, :, :output_dim]
    return normalized_coverage_features, coverage_features_mean, coverage_features_std

def LoadActions(path):
    actions = LoadTensor(f"{path}/normalized_actions.pt")
    actions_mean = LoadTensor(f"{path}/../actions_mean.pt")
    actions_std = LoadTensor(f"{path}/../actions_std.pt")
    return actions, actions_mean, actions_std

def LoadRobotPositions(path):
    robot_positions = LoadTensor(f"{path}/robot_positions.pt")
    return robot_positions

def LoadEdgeWeights(path):
    edge_weights = LoadTensor(f"{path}/edge_weights.pt")
    edge_weights.to_dense()
    return edge_weights

def ToTorchGeometricData(feature, edge_weights):
    edge_weights = edge_weights.to_sparse()
    edge_weights = edge_weights.coalesce()
    edge_index = edge_weights.indices().long()
    weights = torch.reciprocal(edge_weights.values().float())
    data = torch_geometric.data.Data(
            x=feature,
            edge_index=edge_index,
            edge_weight=weights,
            )
    return data

def ToTorchGeometricDataRobotPositions(feature, robot_positions):
    x = robot_positions
    dist_matrix = torch.cdist(x, x, 2)
    dist_matrix[dist_matrix > 256] = 0
    C = (2048 **2)/(dist_matrix.shape[0] ** 2)
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
