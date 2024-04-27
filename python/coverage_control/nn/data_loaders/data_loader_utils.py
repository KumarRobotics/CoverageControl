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

import torch
import torch_geometric
from coverage_control import IOUtils

__all__ = ["DataLoaderUtils"]


## @ingroup python_api
class DataLoaderUtils:
    """
    Class to provide utility functions to load tensors and configuration files
    """

    @staticmethod
    def load_maps(path: str, use_comm_map: bool = False) -> torch.tensor:
        """
        Function to load maps stored as tensors

        The maps are stored as tensors in the following format:
        - {path}/local_maps.pt: Local maps
        - {path}/obstacle_maps.pt: Obstacle maps
        - {path}/comm_maps.pt: Communication maps (if use_comm_map is True)

        Args:
            path (str): Path to the directory containing the maps
            use_comm_map (bool): Whether to load the communication map

        Returns:
            maps: The loaded maps

        """
        local_maps = IOUtils.load_tensor(f"{path}/local_maps.pt")
        local_maps = local_maps.to_dense().unsqueeze(2)
        obstacle_maps = IOUtils.load_tensor(f"{path}/obstacle_maps.pt")
        obstacle_maps = obstacle_maps.to_dense().unsqueeze(2)

        if use_comm_map:
            comm_maps = IOUtils.load_tensor(f"{path}/comm_maps.pt")
            comm_maps = comm_maps.to_dense()
            # comm_maps = (comm_maps * 256 + 256)/512
            maps = torch.cat([local_maps, comm_maps, obstacle_maps], 2)
        else:
            maps = torch.cat([local_maps, obstacle_maps], 2)

        return maps

    @staticmethod
    def load_features(
        path: str, output_dim: int = None
    ) -> tuple[torch.tensor, torch.tensor, torch.tensor]:
        """
        Function to load normalized features

        The features are stored as tensors in the following format:
        - {path}/normalized_coverage_features.pt: Normalized coverage features
        - {path}/../coverage_features_mean.pt: Mean of the coverage features
        - {path}/../coverage_features_std.pt: Standard deviation of the coverage features

        Args:
            path (str): Path to the directory containing the features
            output_dim (int): Output dimension of the features

        Returns:
            features: The loaded features
            features_mean: Mean of the features
            features_std: Standard deviation of the features
        """
        normalized_coverage_features = IOUtils.load_tensor(
            f"{path}/normalized_coverage_features.pt"
        )
        coverage_features_mean = IOUtils.load_tensor(
            f"{path}/../coverage_features_mean.pt"
        )
        coverage_features_std = IOUtils.load_tensor(
            f"{path}/../coverage_features_std.pt"
        )

        if output_dim is not None:
            normalized_coverage_features = normalized_coverage_features[
                :, :, :output_dim
            ]

        return (
            normalized_coverage_features,
            coverage_features_mean,
            coverage_features_std,
        )

    @staticmethod
    def load_actions(path: str) -> tuple[torch.tensor, torch.tensor, torch.tensor]:
        """
        Function to load normalized actions

        The actions are stored as tensors in the following format:
        - {path}/normalized_actions.pt: Normalized actions
        - {path}/../actions_mean.pt: Mean of the actions
        - {path}/../actions_std.pt: Standard deviation of the actions

        Args:
            path (str): Path to the directory containing the actions

        Returns:
            actions: The loaded actions
            actions_mean: Mean of the actions
            actions_std: Standard deviation of the actions

        """
        actions = IOUtils.load_tensor(f"{path}/normalized_actions.pt")
        actions_mean = IOUtils.load_tensor(f"{path}/../actions_mean.pt")
        actions_std = IOUtils.load_tensor(f"{path}/../actions_std.pt")

        return actions, actions_mean, actions_std

    @staticmethod
    def load_robot_positions(path: str) -> torch.tensor:
        """
        Function to load robot positions

        The robot positions are stored as tensors in the following format:
        - {path}/robot_positions.pt: Robot positions

        Args:
            path (str): Path to the directory containing the robot positions

        Returns:
            robot_positions: The loaded robot positions

        """
        robot_positions = IOUtils.load_tensor(f"{path}/robot_positions.pt")

        return robot_positions

    @staticmethod
    def load_edge_weights(path: str) -> torch.tensor:
        """
        Function to load edge weights

        The edge weights are stored as tensors in the following format:
        - {path}/edge_weights.pt: Edge weights

        Args:
            path (str): Path to the directory containing the edge weights

        Returns:
            edge_weights: The loaded edge weights

        """
        edge_weights = IOUtils.load_tensor(f"{path}/edge_weights.pt")
        edge_weights.to_dense()

        return edge_weights

    @staticmethod
    def to_torch_geometric_data(
        feature: torch.tensor, edge_weights: torch.tensor, pos: torch.tensor = None
    ) -> torch_geometric.data.Data:
        """
        The function converts the feature, edge_weights and pos to a torch_geometric.data.Data object
        This is essential for using the data with the PyTorch Geometric library

        Args:
            feature (torch.tensor): The feature tensor
            edge_weights (torch.tensor): The edge weights tensor
            pos (torch.tensor): The position tensor

        Returns:
            data: The torch_geometric.data.Data object
            data.x: The feature tensor
            data.edge_index: The edge index tensor
            data.edge_weight: The edge weight tensor
            data.pos: The position tensor (if pos is not None)

        """
        # senders, receivers = numpy.nonzero(edge_weights)
        # weights = edge_weights[senders, receivers]
        # edge_index = numpy.stack([senders, receivers])
        edge_weights = edge_weights.to_sparse()
        edge_weights = edge_weights.coalesce()
        edge_index = edge_weights.indices().long()
        weights = edge_weights.values().float()
        # weights = torch.reciprocal(edge_weights.values().float())

        if pos is None:
            data = torch_geometric.data.Data(
                x=feature,
                edge_index=edge_index.clone(),
                edge_weight=weights.clone(),
            )
        else:
            data = torch_geometric.data.Data(
                x=feature,
                edge_index=edge_index.clone(),
                edge_weight=weights.clone(),
                pos=pos.clone(),
            )

        return data
