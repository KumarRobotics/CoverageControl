import yaml
import os
import torch
import src.data_loaders.data_loader_utils as dl_utils
from torch_geometric.data import Dataset


class LocalMapCNNDataset(Dataset):
    """
    Dataset for CNN training
    """
    def __init__(self, data_dir, stage, use_comm_map, output_dim):
        super(LocalMapCNNDataset, self).__init__(None, None, None, None)

        self.stage = stage
        self.output_dim = output_dim

        # maps has shape (num_samples, num_robots, num_channels, image_size, image_size)
        self.maps = dl_utils.LoadMaps(f"{data_dir}/{stage}", use_comm_map)
        num_channels = self.maps.shape[2]
        image_size = self.maps.shape[3]

        self.maps = self.maps.view(-1, num_channels, image_size, image_size)
        self.dataset_size = self.maps.shape[0]

        self.targets, self.targets_mean, self.targets_std = dl_utils.LoadFeatures(f"{data_dir}/{stage}", output_dim)
        self.targets = self.targets.view(-1, self.targets.shape[2])
        self.targets = self.targets[:, :output_dim]

    def len(self):
        return self.dataset_size

    def get(self, idx):
        maps = self.maps[idx]
        target = self.targets[idx]
        return maps, target

class LocalMapGNNDataset(Dataset):
    """
    Dataset for hybrid CNN-GNN training
    """
    def __init__(self, data_dir, stage, use_comm_map):
        super(LocalMapGNNDataset, self).__init__(None, None, None, None)

        self.stage = stage

        self.maps = dl_utils.LoadMaps(f"{data_dir}/{stage}", use_comm_map)
        self.dataset_size = self.maps.shape[0]

        self.targets, self.targets_mean, self.targets_std = dl_utils.LoadActions(f"{data_dir}/{stage}")
        self.edge_weights = dl_utils.LoadEdgeWeights(f"{data_dir}/{stage}")

    def len(self):
        return self.dataset_size

    def get(self, idx):
        data = dl_utils.ToTorchGeometricData(self.maps[idx], self.edge_weights[idx], self.targets[idx])
        return data, data.y

class VoronoiGNNDataset(Dataset):
    """
    Dataset for non-hybrid GNN training
    """
    def __init__(self, data_dir, stage, output_dim):
        super(VoronoiGNNDataset, self).__init__(None, None, None, None)

        self.stage = stage
        self.output_dim = output_dim

        self.features = dl_utils.LoadFeatures(f"{data_dir}/{stage}", output_dim)
        self.dataset_size = self.features.shape[0]
        self.targets, self.targets_mean, self.targets_std = dl_utils.LoadActions(f"{data_dir}/{stage}")
        self.edge_weights = dl_utils.LoadEdgeWeights(f"{data_dir}/{stage}")


    def len(self):
        return self.dataset_size

    def get(self, idx):
        data = dl_utils.ToTorchGeometricData(self.features[idx], self.edge_weights[idx], self.targets[idx])
        return data, data.y
