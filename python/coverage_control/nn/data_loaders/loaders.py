#  This file is part of the CoverageControl library
#
#  Author: Saurav Agarwal
#  Contact: sauravag@seas.upenn.edu, agr.saurav1@gmail.com
#  Repository: https://github.com/KumarRobotics/CoverageControl
#
#  Copyright (c) 2024, Saurav Agarwal
#
#  The CoverageControl library is free software: you can redistribute it and/or
#  modify it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or (at your
#  option) any later version.
#
#  The CoverageControl library is distributed in the hope that it will be
#  useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
#  Public License for more details.
#
#  You should have received a copy of the GNU General Public License along with
#  CoverageControl library. If not, see <https://www.gnu.org/licenses/>.

"""
Module for loading datasets
"""

import torch
from coverage_control import IOUtils
from torch_geometric.data import Dataset

from ...coverage_env_utils import CoverageEnvUtils
from .data_loader_utils import DataLoaderUtils


## @ingroup python_api
class LocalMapCNNDataset(Dataset):
    """
    Dataset for CNN training
    """

    def __init__(
            self,
            data_dir: str,
            stage: str,
            use_comm_map: bool,
            output_dim: int,
            preload: bool = True,
            ):
        super().__init__(None, None, None, None)
        """
        Constructor for the LocalMapCNNDataset class
        Args:
            data_dir (str): Directory containing the data
            stage (str): Stage of the data (train, val, test)
            use_comm_map (bool): Whether to use communication maps
            output_dim (int): Dimension of the output
            preload (bool): Whether to preload the data
        """

        self.stage = stage
        self.data_dir = data_dir
        self.output_dim = output_dim
        self.use_comm_map = use_comm_map

        if preload is True:
            self.load_data()

    def len(self):
        return self.dataset_size

    def get(self, idx):
        maps = self.maps[idx]
        target = self.targets[idx]

        return maps, target

    def load_data(self):
        """
        Load the data from the data directory
        """
        # maps has shape (num_samples, num_robots, nuimage_size, image_size)
        self.maps = DataLoaderUtils.load_maps(
                f"{self.data_dir}/{self.stage}", self.use_comm_map
                )
        num_channels = self.maps.shape[2]
        image_size = self.maps.shape[3]

        self.maps = self.maps.view(-1, num_channels, image_size, image_size)
        self.dataset_size = self.maps.shape[0]

        # self.targets, self.targets_mean, self.targets_std = DataLoaderUtils.load_features(f"{self.data_dir}/{self.stage}", self.output_dim)
        self.targets, self.targets_mean, self.targets_std = (
                DataLoaderUtils.load_actions(f"{self.data_dir}/{self.stage}")
                )
        self.targets = self.targets.view(-1, self.targets.shape[2])

## @ingroup python_api
class CNNGNNDataset(Dataset):
    """
    Dataset for hybrid CNN-GNN training
    """

    def __init__(self, data_dir, stage, use_comm_map, world_size):
        super().__init__(None, None, None, None)

        self.stage = stage

        self.maps = DataLoaderUtils.load_maps(f"{data_dir}/{stage}", use_comm_map)
        self.dataset_size = self.maps.shape[0]

        self.targets, self.targets_mean, self.targets_std = (
                DataLoaderUtils.load_actions(f"{data_dir}/{stage}")
                )
        self.edge_weights = DataLoaderUtils.load_edge_weights(f"{data_dir}/{stage}")

        self.robot_positions = DataLoaderUtils.load_robot_positions(
                f"{data_dir}/{stage}"
                )
        self.robot_positions = (self.robot_positions + world_size / 2) / world_size

        # Print the details of the dataset with device information
        print(f"Dataset: {self.stage} | Size: {self.dataset_size}",
              f"Coverage Maps: {self.maps.shape}",
              f"Targets: {self.targets.shape}",
              f"Robot Positions: {self.robot_positions.shape}",
              f"Edge Weights: {self.edge_weights.shape}",
              )

    def len(self):
        return self.dataset_size

    def get(self, idx):
        data = DataLoaderUtils.to_torch_geometric_data(
                self.maps[idx], self.edge_weights[idx], self.robot_positions[idx]
                )
        # data = CoverageEnvUtils.GetTorchGeometricDataRobotPositions(self.maps[idx], self.robot_positions[idx])
        targets = self.targets[idx]

        if targets.dim == 3:
            targets = targets.view(-1, targets.shape[-1])

        return data, targets


## @ingroup python_api
class VoronoiGNNDataset(Dataset):
    """
    Dataset for non-hybrid GNN training
    """

    def __init__(self, data_dir, stage, output_dim):
        super().__init__(None, None, None, None)

        self.stage = stage
        self.output_dim = output_dim

        self.features = DataLoaderUtils.load_features(f"{data_dir}/{stage}", output_dim)
        self.dataset_size = self.features[0].shape[0]
        self.targets, self.targets_mean, self.targets_std = (
                DataLoaderUtils.load_actions(f"{data_dir}/{stage}")
                )
        self.edge_weights = DataLoaderUtils.load_edge_weights(f"{data_dir}/{stage}")

    def len(self):
        return self.dataset_size

    def get(self, idx):
        data = DataLoaderUtils.to_torch_geometric_data(
                self.features[idx], self.edge_weights[idx], self.targets[idx]
                )

        return data, data.y
