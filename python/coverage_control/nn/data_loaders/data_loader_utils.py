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

import os
import yaml
import tomllib
import torch
import torch_geometric

## @ingroup python_api
class DataLoaderUtils:
    """
    Class to provide utility functions to load tensors and configuration files
    """

    def load_tensor(path):
        """
        Function to load a tensor from a file
        Can load tensors from jit script format files

        Args:
            path (str): Path to the file

        Returns:
            tensor: The loaded tensor
            None: If the file does not exist

        Raises:
            FileNotFoundError: If the file does not exist
        """
        # Throw error if path does not exist
        if not os.path.exists(path):
            raise FileNotFoundError(f"DataLoaderUtils::load_tensor: File not found: {path}")
        # Load data
        data = torch.load(path)
        # Extract tensor if data is in jit script format
        if isinstance(data, torch.jit.ScriptModule):
            tensor = list(data.parameters())[0]
        else:
            tensor = data
        return tensor

    def load_yaml(path):
        """
        Function to load a yaml file

        Args:
            path (str): Path to the file

        Returns:
            data: The loaded data
        Raises:
            FileNotFoundError: If the file does not exist
        """


        # Throw error if path does not exist
        if not os.path.exists(path):
            raise FileNotFoundError(f"DataLoaderUtils::load_yaml File not found: {path}")
        # Load data
        with open(path, "r") as f:
            data = yaml.load(f, Loader=yaml.FullLoader)
        return data

    def LoadToml(path):
        # Throw error if path does not exist
        if not os.path.exists(path):
            raise FileNotFoundError(f"data_loader_utils::LoadToml: File not found: {path}")
        # Load data
        with open(path, "rb") as f:
            data = tomllib.load(f)
        return data

    def LoadMaps(path, use_comm_map):
        local_maps = load_tensor(f"{path}/local_maps.pt")
        local_maps = local_maps.to_dense().unsqueeze(2)
        obstacle_maps = load_tensor(f"{path}/obstacle_maps.pt")
        obstacle_maps = obstacle_maps.to_dense().unsqueeze(2)

        if use_comm_map:
            comm_maps = load_tensor(f"{path}/comm_maps.pt")
            comm_maps = comm_maps.to_dense()
            # comm_maps = (comm_maps * 256 + 256)/512
            maps = torch.cat([local_maps, comm_maps, obstacle_maps], 2)
        else:
            maps = torch.cat([local_maps, obstacle_maps], 2)
        return maps

    def LoadFeatures(path, output_dim = None):
        normalized_coverage_features = load_tensor(f"{path}/normalized_coverage_features.pt")
        coverage_features_mean = load_tensor(f"{path}/../coverage_features_mean.pt")
        coverage_features_std = load_tensor(f"{path}/../coverage_features_std.pt")
        if output_dim is not None:
            normalized_coverage_features = normalized_coverage_features[:, :, :output_dim]
        return normalized_coverage_features, coverage_features_mean, coverage_features_std

    def LoadActions(path):
        actions = load_tensor(f"{path}/normalized_actions.pt")
        actions_mean = load_tensor(f"{path}/../actions_mean.pt")
        actions_std = load_tensor(f"{path}/../actions_std.pt")
        return actions, actions_mean, actions_std

    def LoadRobotPositions(path):
        robot_positions = load_tensor(f"{path}/robot_positions.pt")
        return robot_positions

    def LoadEdgeWeights(path):
        edge_weights = load_tensor(f"{path}/edge_weights.pt")
        edge_weights.to_dense()
        return edge_weights

    def ToTorchGeometricData(feature, edge_weights, pos = None):
        # senders, receivers = numpy.nonzero(edge_weights)
        # weights = edge_weights[senders, receivers]
        # edge_index = numpy.stack([senders, receivers])
        edge_weights = edge_weights.to_sparse()
        edge_weights = edge_weights.coalesce()
        edge_index = edge_weights.indices().long()
        weights = edge_weights.values().float()
        # weights = torch.reciprocal(edge_weights.values().float())
        if pos == None:
            data = torch_geometric.data.Data(
                    x=feature,
                    edge_index=edge_index.clone().detach(),
                    edge_weight=weights.clone().detach()
                    )
        else:
            data = torch_geometric.data.Data(
                    x=feature,
                    edge_index=edge_index.clone().detach(),
                    edge_weight=weights.clone().detach(),
                    pos=pos.clone().detach()
                    )
        return data
