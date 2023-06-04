import yaml
import os
import torch
import torch_geometric

class DataLoaderUtils():
    """
    Simple class for common data loader utilities
    """
    def __init__(self):
        pass

    """
    Function to load a tensor from a file
    Checks if the file exists, if not, returns None
    Checks if the loaded data is a tensor or is in jit script format
    """
    @staticmethod
    def load_tensor(path):
        if not os.path.exists(path):
            return None
        data = torch.load(path)
        # Extract tensor if data is in jit script format
        if isinstance(data, torch.jit.ScriptModule):
            tensor = list(data.parameters())[0]
        else:
            tensor = data
        return tensor

    @staticmethod
    def load_yaml(path):
        with open(path, "r") as f:
            data = yaml.load(f, Loader=yaml.FullLoader)
        return data

    @staticmethod
    def LoadMaps(path, use_comm_map):
        local_maps = load_tensor(f"{path}/local_maps.pt")
        image_size = local_maps.shape[2]
        local_maps = local_maps.unsqueeze(2).view(-1, 1, image_size, image_size)
        obstacle_maps = load_tensor(f"{path}/obstacle_maps.pt")
        obstacle_maps = obstacle_maps.to_dense().unsqueeze(2).view(-1, 1, image_size, image_size)

        if use_comm_map:
            comm_maps = load_tensor(f"{path}/comm_maps.pt")
            comm_maps = comm_maps.to_dense().view(-1, 2, image_size, image_size)
            maps = torch.cat([local_maps, comm_maps, obstacle_maps], 1)
        else:
            maps = torch.cat([local_maps, obstacle_maps], 1)
        return maps

    @staticmethod
    def LoadFeatures(path, output_dim = None):
        normalized_coverage_features = load_tensor(f"{path}/normalized_coverage_features.pt")
        coverage_features_mean = load_tensor(f"{path}/coverage_features_mean.pt")
        coverage_features_std = load_tensor(f"{path}/coverage_features_std.pt")
        if output_dim is not None:
            normalized_coverage_features = normalized_coverage_features[::, :output_dim]
        return normalized_coverage_features, coverage_features_mean, coverage_features_std

    @staticmethod
    def LoadActions(path):
        actions = load_tensor(f"{path}/actions.pt")
        actions_mean = load_tensor(f"{path}/actions_mean.pt")
        actions_std = load_tensor(f"{path}/actions_std.pt")
        return actions, actions_mean, actions_std

    @staticmethod
    def LoadEdgeWeights(path):
        edge_weights = load_tensor(f"{path}/edge_weights.pt")
        edge_weights.to_dense()
        return edge_weights

    @staticmethod
    def ToTorchGeometricData(feature, edge_weights, target):
        edge_weights = edge_weights.to_sparse()
        edge_index = edge_weights.indices()
        weights = edge_weights.values()
        data = torch_geometric.data.Data(
                x=feature,
                edge_index=torch.tensor(edge_index).long(),
                edge_weight=torch.tensor(weights).float(),
                y=target
                )
        return data
