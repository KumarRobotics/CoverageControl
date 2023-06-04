import yaml
import os
import torch
from torch_geometric.data import Dataset

class LocalMapCNNDataset(Dataset, DataLoaderUtils):
    """
    Dataset for CNN training
    """
    def __init__(self, data_dir, stage, use_comm_map, ouput_dim):
        super(Dataset, self).__init__(None, None, None, None)

        self.stage = stage
        self.output_dim = output_dim

        self.maps = self.LoadMaps(f"{data_dir}/{stage}", use_comm_map)
        self.dataset_size = self.maps.shape[0]

        self.targets, self.targets_mean, self.targets_std = self.LoadFeatures(f"{data_dir}/{stage}", output_dim)
        self.targets = self.targets.view(-1, self.targets.shape[2])
        self.targets = self.targets[:, :output_dim]

    def len(self):
        return self.dataset_size

    def get(self, idx):
        maps = self.maps[idx]
        target = self.targets[idx]
        return maps, target

class LocalMapGNNDataset(Dataset, DataLoaderUtils):
    """
    Dataset for hybrid CNN-GNN training
    """
    def __init__(self, data_dir, stage, use_comm_map):
        super(Dataset, self).__init__(None, None, None, None)

        self.stage = stage

        self.maps = self.LoadMaps(f"{data_dir}/{stage}", use_comm_map)
        self.dataset_size = self.maps.shape[0]

        self.targets, self.targets_mean, self.targets_std = self.LoadActions(f"{data_dir}/{stage}")
        self.edge_weights = self.LoadEdgeWeights(f"{data_dir}/{stage}")

    def len(self):
        return self.dataset_size

    def get(self, idx):
        data = self.ToTorchGeometricData(self.maps[idx], self.edge_weights[idx], self.targets[idx])
        return data

class VoronoiGNNDataset(Dataset, DataLoaderUtils):
    """
    Dataset for non-hybrid GNN training
    """
    def __init__(self, data_dir, stage, output_dim):
        super(Dataset, self).__init__(None, None, None, None)

        self.stage = stage
        self.output_dim = output_dim

        self.features = self.LoadFeatures(f"{data_dir}/{stage}", output_dim)
        self.dataset_size = self.features.shape[0]
        self.targets, self.targets_mean, self.targets_std = self.LoadActions(f"{data_dir}/{stage}")
        self.edge_weights = self.LoadEdgeWeights(f"{data_dir}/{stage}")


    def len(self):
        return self.dataset_size

    def get(self, idx):
        data = self.ToTorchGeometricData(self.features[idx], self.edge_weights[idx], self.targets[idx])
        return data
