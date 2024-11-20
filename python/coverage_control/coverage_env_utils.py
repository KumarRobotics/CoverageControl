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
Utility functions for coverage environment
"""

## @file coverage_env_utils.py
#  @brief Utility functions for coverage environment

import math

import numpy

# import cv2
import torch
import torch_geometric
import torchvision

from .core import CoverageSystem, DblVector, DblVectorVector, Parameters, PointVector


## @ingroup python_api
class CoverageEnvUtils:
    """
    Class for utility functions for coverage environment
    """

    @staticmethod
    def to_tensor(data: object) -> torch.Tensor:
        """
        Converts various types of data to torch.Tensor

        Can accept the following types:
        - numpy.ndarray
        - PointVector
        - DblVectorVector
        - DblVector

        Args:
            data: input data

        Returns:
            torch.Tensor: converted data

        Raises:
            ValueError: if data type is not supported

        """

        if isinstance(data, numpy.ndarray):
            return torch.from_numpy(numpy.copy(data.astype(numpy.float32)))

        if isinstance(data, PointVector):
            data_tensor = torch.Tensor(len(data), 2)

            for i, _ in enumerate(data):
                data_tensor[i] = CoverageEnvUtils.to_tensor(data[i])

            return data_tensor

        if isinstance(data, DblVectorVector):
            data_tensor = torch.Tensor(len(data))

            for i, _ in enumerate(data):
                data_tensor[i] = CoverageEnvUtils.to_tensor(data[i])

            return data_tensor

        if isinstance(data, DblVector):
            data_tensor = torch.Tensor(len(data))

            for i, _ in enumerate(data):
                data_tensor[i] = float(data[i])

            return data_tensor
        raise ValueError(f"Unknown data type: {type(data)}")

    @staticmethod
    def get_raw_local_maps(env: CoverageSystem, params: Parameters) -> torch.Tensor:
        """
        Get raw local maps

        Args:
            env: coverage environment
            params: parameters

        Returns:
            torch.Tensor: raw local maps

        """
        local_maps = torch.zeros(
            (env.GetNumRobots(), params.pLocalMapSize, params.pLocalMapSize)
        )

        for r_idx in range(env.GetNumRobots()):
            local_maps[r_idx] = CoverageEnvUtils.to_tensor(env.GetRobotLocalMap(r_idx))

        return local_maps

    @staticmethod
    def get_raw_obstacle_maps(env: CoverageSystem, params: Parameters) -> torch.Tensor:
        """
        Get raw obstacle maps

        Args:
            env: coverage environment
            params: parameters

        Returns:
            torch.Tensor: raw obstacle maps

        """
        obstacle_maps = torch.zeros(
            (env.GetNumRobots(), params.pLocalMapSize, params.pLocalMapSize)
        )

        for r_idx in range(env.GetNumRobots()):
            obstacle_maps[r_idx] = CoverageEnvUtils.to_tensor(
                env.GetRobotObstacleMap(r_idx)
            )

        return obstacle_maps

    @staticmethod
    def get_communication_maps(
        env: CoverageSystem, params: Parameters, map_size: int
    ) -> torch.Tensor:
        """
        Generate communication maps from positions

        Communication maps are composed of two channels.
        Each channnel has non-zero values for cells that correspond to the relative positions of the neighbors.
        For the first channel, the value is the x-coordinate of the relative position divided by the communication range.
        Similarly, the y-coordinte is used for the second channel.

        Args:
            env: coverage environment
            params: parameters
            map_size: size of the map

        Returns:
            torch.Tensor: communication maps
        """
        num_robots = env.GetNumRobots()

        comm_maps = torch.zeros((num_robots, 2, map_size, map_size))

        for r_idx in range(num_robots):
            neighbors_pos = CoverageEnvUtils.to_tensor(
                env.GetRelativePositonsNeighbors(r_idx)
            )
            scaled_indices = torch.round(
                neighbors_pos
                * map_size
                / (params.pCommunicationRange * params.pResolution * 2.0)
                + (map_size / 2.0 - params.pResolution / 2.0)
            )
            # comm_range_mask = relative_dist[r_idx] < params.pCommunicationRange
            # scaled_indices = scaled_relative_pos[r_idx][comm_range_mask]
            indices = torch.transpose(scaled_indices, 1, 0)
            indices = indices.long()
            values = neighbors_pos / params.pCommunicationRange
            # values = values / params.pCommunicationRange
            # values = (values + params.pCommunicationRange) / (2. * params.pCommunicationRange)
            comm_maps[r_idx][0] = torch.sparse_coo_tensor(
                indices, values[:, 0], torch.Size([map_size, map_size])
            ).to_dense()
            comm_maps[r_idx][1] = torch.sparse_coo_tensor(
                indices, values[:, 1], torch.Size([map_size, map_size])
            ).to_dense()

        return comm_maps
        # positions = env.GetRobotPositions()
        # robot_positions = CoverageEnvUtils.to_tensor(env.GetRobotPositions())
        # relative_pos = robot_positions.unsqueeze(0) - robot_positions.unsqueeze(1)
        # scaled_relative_pos = torch.round(relative_pos * map_size / (params.pCommunicationRange * params.pResolution * 2.) + (map_size / 2. - params.pResolution / 2.))
        # relative_dist = relative_pos.norm(2, 2)
        # diagonal_mask = torch.eye(num_robots).to(torch.bool)
        # relative_dist.masked_fill_(diagonal_mask, params.pCommunicationRange + 1)

    @staticmethod
    def resize_maps(maps: torch.Tensor, resized_map_size: int) -> torch.Tensor:
        """
        Resize maps to a given size
        Uses bilinear interpolation from torchvision.transforms.functional.resize
        Options: antialias=True

        Args:
            maps: input maps
            resized_map_size: size of the resized maps

        Returns:
            torch.Tensor: resized maps

        """
        shape = maps.shape
        maps = maps.view(-1, maps.shape[-2], maps.shape[-1])
        maps = torchvision.transforms.functional.resize(
            maps,
            (resized_map_size, resized_map_size),
            interpolation=torchvision.transforms.InterpolationMode.BILINEAR,
            antialias=True,
        )
        maps = maps.view(shape[:-2] + maps.shape[-2:])

        return maps

    @staticmethod
    def get_maps(
        env: CoverageSystem,
        params: Parameters,
        resized_map_size: int,
        use_comm_map: bool,
    ) -> torch.Tensor:
        """
        Get maps for the coverage environment

        Args:
            env: coverage environment
            params: parameters
            resized_map_size: size of the resized maps
            use_comm_map: whether to use communication maps

        Returns:
            torch.Tensor: maps

        """

        num_robots = env.GetNumRobots()
        raw_local_maps = CoverageEnvUtils.get_raw_local_maps(env, params)
        resized_local_maps = CoverageEnvUtils.resize_maps(
            raw_local_maps, resized_map_size
        )
        raw_obstacle_maps = CoverageEnvUtils.get_raw_obstacle_maps(env, params)
        resized_obstacle_maps = CoverageEnvUtils.resize_maps(
            raw_obstacle_maps, resized_map_size
        )

        if use_comm_map:
            comm_maps = env.GetCommunicationMaps(resized_map_size)
            # comm_maps = torch.tensor(numpy.array(env.GetCommunicationMaps(resized_map_size)), dtype=torch.float32).reshape(num_robots, 2, resized_map_size, resized_map_size)
            comm_maps = CoverageEnvUtils.get_communication_maps(
                env, params, resized_map_size
            )
            maps = torch.cat(
                [
                    resized_local_maps.unsqueeze(1),
                    comm_maps,
                    resized_obstacle_maps.unsqueeze(1),
                ],
                1,
            )
        else:
            maps = torch.cat(
                [resized_local_maps.unsqueeze(1), resized_obstacle_maps.unsqueeze(1)], 1
            )

        return maps

    @staticmethod
    def get_voronoi_features(env: CoverageSystem) -> torch.Tensor:
        """
        Get voronoi features

        Args:
            env: coverage environment

        Returns:
            torch.Tensor: voronoi features
        """
        features = env.GetRobotVoronoiFeatures()
        tensor_features = torch.zeros((len(features), len(features[0])))

        for r_idx, _ in enumerate(features):
            tensor_features[r_idx] = CoverageEnvUtils.to_tensor(features[r_idx])

        return tensor_features

    @staticmethod
    def get_robot_positions(env: CoverageSystem) -> torch.Tensor:
        """
        Get robot positions

        Args:
            env: coverage environment

        Returns:
            torch.Tensor: robot positions
        """
        robot_positions = CoverageEnvUtils.to_tensor(env.GetRobotPositions())

        return robot_positions

    @staticmethod
    def get_weights(env: CoverageSystem, params: Parameters) -> torch.Tensor:
        """
        Get edge weights for the communication graph

        Args:
            env: coverage environment
            params: parameters

        Returns:
            torch.Tensor: edge weights
        """
        onebyexp = 1.0 / math.exp(1.0)
        robot_positions = CoverageEnvUtils.to_tensor(env.GetRobotPositions())
        pairwise_distances = torch.cdist(robot_positions, robot_positions, 2)
        edge_weights = torch.exp(
            -(pairwise_distances.square())
            / (params.pCommunicationRange * params.pCommunicationRange)
        )
        edge_weights.masked_fill_(edge_weights < onebyexp, 0)
        edge_weights.fill_diagonal_(0)

        return edge_weights

    @staticmethod
    def get_torch_geometric_data(
        env: CoverageSystem,
        params: Parameters,
        use_cnn: bool,
        use_comm_map: bool,
        map_size: int,
    ) -> torch_geometric.data.Data:
        """
        Get torch geometric data
        In this function, the edge weights are binary

        Args:
            env: coverage environment
            params: parameters
            use_cnn: whether to use CNN
            use_comm_map: whether to use communication maps
            map_size: size of the maps

        Returns:
            torch_geometric.data.Data: torch geometric data

        """

        if use_cnn:
            features = CoverageEnvUtils.get_maps(env, params, map_size, use_comm_map)
        else:
            features = CoverageEnvUtils.get_voronoi_features(env)
        edge_weights = CoverageEnvUtils.get_weights(env, params).to_sparse().coalesce()
        edge_index = edge_weights.indices().long()
        weights = edge_weights.values().float()
        pos = CoverageEnvUtils.get_robot_positions(env)
        pos = (pos + params.pWorldMapSize / 2.0) / params.pWorldMapSize
        data = torch_geometric.data.Data(
            x=features,
            edge_index=edge_index.clone().detach(),
            edge_weight=weights.clone().detach(),
            pos=pos.clone().detach(),
        )

        return data

    # Legacy maps which gives decent results
    # Trying to move away from this
    # @staticmethod
    # def get_stable_maps(env, params, resized_map_size):
    #     robot_positions = CoverageEnvUtils.to_tensor(env.GetRobotPositions())
    #     num_robots = env.GetNumRobots()
    #     maps = torch.empty((num_robots, 4, resized_map_size, resized_map_size))
    #     h_vals = torch.linspace(1.0, -1.0, maps.shape[-2]+1)
    #     h_vals = (h_vals[1:] + h_vals[:-1])/2
    #     w_vals = torch.linspace(-1.0, 1.0, maps.shape[-1]+1)
    #     w_vals = (w_vals[1:] + w_vals[:-1])/2
    #     heatmap_x = torch.stack([h_vals] * maps.shape[-1], dim=1)/100
    #     heatmap_y = torch.stack([w_vals] * maps.shape[-2], dim=0)/100
    #     for r_idx in range(num_robots):
    #         local_map = env.GetRobotLocalMap(r_idx)
    #         resized_local_map = cv2.resize(local_map, dsize=(resized_map_size, resized_map_size), interpolation=cv2.INTER_AREA)
    #         maps[r_idx][0] = torch.tensor(resized_local_map).float()

    #         comm_map = env.GetCommunicationMap(r_idx)
    #         filtered_comm_map = gaussian_filter(comm_map, sigma=(3,3), order=0)
    #         resized_comm_map = torch.tensor(cv2.resize(numpy.array(filtered_comm_map), dsize=(resized_map_size, resized_map_size), interpolation=cv2.INTER_AREA)).float()
    #         maps[r_idx][1] = resized_comm_map

    #         maps[r_idx][2] = heatmap_x
    #         maps[r_idx][3] = heatmap_y

    #     return maps


    # Legacy edge weights used in previous research
    # The weights are proportional to the distance
    # Trying to move away from this
    # @staticmethod
    # def robot_positions_to_edge_weights(
    #     robot_positions: PointVector, world_map_size: int, comm_range: float
    # ) -> torch.Tensor:
    #     """
    #     Convert robot positions to edge weights

    #     Args:
    #         robot_positions: robot positions
    #         world_map_size: size of the world map
    #         comm_range: communication range

    #     Returns:
    #         torch.Tensor: edge weights
    #     """
    #     x = numpy.array(robot_positions)
    #     s_mat = distance_matrix(x, x)
    #     s_mat[s_mat > comm_range] = 0
    #     c_mat = (world_map_size**2) / (s_mat.shape[0] ** 2)
    #     c_mat = 3 / c_mat
    #     graph_obs = c_mat * s_mat

    #     return graph_obs

