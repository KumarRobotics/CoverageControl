import yaml
import os
import torch
import CoverageControlTorch
import CoverageControlTorch.data_loaders.data_loader_utils as dl_utils
import CoverageControlTorch.utils.coverage_system as coverage_system
from torch_geometric.data import Dataset


class LocalMapCNNDataset(Dataset):
    """
    Dataset for CNN training
    """
    def __init__(self, data_dir, stage, use_comm_map, output_dim, preload=True):
        super(LocalMapCNNDataset, self).__init__(None, None, None, None)

        self.stage = stage
        self.data_dir = data_dir
        self.output_dim = output_dim
        self.use_comm_map = use_comm_map
        if preload == True:
            self.LoadData()

    def len(self):
        return self.dataset_size

    def get(self, idx):
        maps = self.maps[idx]
        target = self.targets[idx]
        return maps, target

    def LoadData(self):
        # maps has shape (num_samples, num_robots, nuimage_size, image_size)
        self.maps = dl_utils.LoadMaps(f"{self.data_dir}/{self.stage}", self.use_comm_map)
        num_channels = self.maps.shape[2]
        image_size = self.maps.shape[3]

        self.maps = self.maps.view(-1, num_channels, image_size, image_size)
        self.dataset_size = self.maps.shape[0]

        self.targets, self.targets_mean, self.targets_std = dl_utils.LoadFeatures(f"{self.data_dir}/{self.stage}", self.output_dim)
        self.targets = self.targets.view(-1, self.targets.shape[2])

class LocalMapGNNDataset(Dataset):
    """
    Dataset for hybrid CNN-GNN training
    """
    def __init__(self, data_dir, stage):
        super(LocalMapGNNDataset, self).__init__(None, None, None, None)

        self.stage = stage

        # Coverage maps is of shape (num_samples, num_robots, 2, image_size, image_size)
        self.coverage_maps = dl_utils.LoadTensor(f"{data_dir}/{stage}/coverage_maps.pt")
        self.num_robots = self.coverage_maps.shape[1]
        self.dataset_size = self.coverage_maps.shape[0]
        self.targets, self.targets_mean, self.targets_std = dl_utils.LoadActions(f"{data_dir}/{stage}")
        self.robot_positions = dl_utils.LoadRobotPositions(f"{data_dir}/{stage}")

        h_vals = torch.linspace(1.0, -1.0, self.coverage_maps.shape[-2]+1)
        h_vals = (h_vals[1:] + h_vals[:-1])/2
        w_vals = torch.linspace(-1.0, 1.0, self.coverage_maps.shape[-1]+1)
        w_vals = (w_vals[1:] + w_vals[:-1])/2
        self.heatmap_x = torch.stack([h_vals]*self.coverage_maps.shape[-1], axis=1)/100
        self.heatmap_y = torch.stack([w_vals]*self.coverage_maps.shape[-2], axis=0)/100


        # Print the details of the dataset with device information
        print(f"Dataset: {self.stage} | Size: {self.dataset_size}")
        print(f"Coverage Maps: {self.coverage_maps.shape} | Device: {self.coverage_maps.device}")
        print(f"Targets: {self.targets.shape} | Device: {self.targets.device}")
        print(f"Robot Positions: {self.robot_positions.shape} | Device: {self.robot_positions.device}")
        print(f"Heatmap X: {self.heatmap_x.shape} | Device: {self.heatmap_x.device}")
        print(f"Heatmap Y: {self.heatmap_y.shape} | Device: {self.heatmap_y.device}")

    def len(self):
        return self.dataset_size

    def get(self, idx):
        # coverage_maps is of shape (num_robots, 2, image_size, image_size)
        coverage_maps = self.coverage_maps[idx]
        # coverage_maps = coverage_maps.view(-1, 2, coverage_maps.shape[-2], coverage_maps.shape[-1])
        # Add heatmaps to coverage maps
        # heatmaps are of shape image_size x image_size
        heatmap_x = torch.stack([self.heatmap_x] * coverage_maps.shape[0])
        heatmap_y = torch.stack([self.heatmap_y] * coverage_maps.shape[0])
        maps = torch.stack([coverage_maps[:,0], coverage_maps[:,1], heatmap_x, heatmap_y], dim=1)
        # maps = maps.view(self.num_robots, 4, maps.shape[-2], maps.shape[-1])

        edge_weights = coverage_system.RobotPositionsToEdgeWeights(self.robot_positions[idx], 2048, 256)
        data = dl_utils.ToTorchGeometricData(maps, edge_weights)
        targets = self.targets[idx]
        return data, targets

class CNNGNNDataset(Dataset):
    """
    Dataset for hybrid CNN-GNN training
    """
    def __init__(self, data_dir, stage, use_comm_map, world_size):
        super(CNNGNNDataset, self).__init__(None, None, None, None)

        self.stage = stage

        self.maps = dl_utils.LoadMaps(f"{data_dir}/{stage}", use_comm_map)
        self.dataset_size = self.maps.shape[0]

        self.targets, self.targets_mean, self.targets_std = dl_utils.LoadActions(f"{data_dir}/{stage}")
        self.edge_weights = dl_utils.LoadEdgeWeights(f"{data_dir}/{stage}")

        self.robot_positions = dl_utils.LoadRobotPositions(f"{data_dir}/{stage}")
        self.robot_positions = (self.robot_positions + world_size/2)/world_size

        # Print the details of the dataset with device information
        print(f"Dataset: {self.stage} | Size: {self.dataset_size}")
        print(f"Maps: {self.maps.shape} | Device: {self.maps.device}")
        print(f"Targets: {self.targets.shape} | Device: {self.targets.device}")
        print(f"Edge Weights: {self.edge_weights.shape} | Device: {self.edge_weights.device}")
        print(f"Targets: {self.targets.shape} | Device: {self.targets.device}")
        print(f"Robot Positions: {self.robot_positions.shape} | Device: {self.robot_positions.device}")


    def len(self):
        return self.dataset_size

    def get(self, idx):
        data = dl_utils.ToTorchGeometricData(self.maps[idx], self.edge_weights[idx], self.robot_positions[idx])
        # data = coverage_system.GetTorchGeometricDataRobotPositions(self.maps[idx], self.robot_positions[idx])
        targets = self.targets[idx]
        if targets.dim == 3:
            targets = targets.view(-1, targets.shape[-1])
        return data, targets

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
