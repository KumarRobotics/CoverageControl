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
import warnings

import coverage_control
import numpy as np
import torch
import torch_geometric
from coverage_control import CoverageEnvUtils

script_dir = os.path.dirname(os.path.realpath(__file__))

params_file = os.path.join(script_dir, "data/params/coverage_control_params.toml")
params = coverage_control.Parameters(params_file)

features_file = os.path.join(script_dir, "data/features")
robot_pos_file = os.path.join(script_dir, "data/robots_positions")

world_idf = coverage_control.WorldIDF(params)
world_map_ref = np.load(os.path.join(script_dir, "data/world_map.npy"))
print(world_map_ref.sum())
world_map = world_idf.GetWorldMap()
world_map = world_map_ref

# world_idf.LoadMap(os.path.join(script_dir, "data/world_map.dat"))
# world_idf = coverage_control.WorldIDF(params, os.path.join(script_dir, features_file))
env = coverage_control.CoverageSystem(params, world_idf, robot_pos_file)

params.pNumRobots = env.GetNumRobots()


def test_to_tensor():
    rand_np = np.random.rand(10, 10).astype(np.float32)
    rand_torch = CoverageEnvUtils.to_tensor(rand_np)
    assert isinstance(rand_torch, torch.Tensor)
    assert rand_torch.shape == (10, 10)
    assert rand_torch.dtype == torch.float32
    is_all_close = np.allclose(rand_np, rand_torch.numpy())
    assert is_all_close
    is_all_equal = np.equal(rand_np, rand_torch.numpy()).all()

    if not is_all_equal and is_all_close:
        warnings.warn("Not all elements are equal, but all elements are close")


def test_to_tensor_device():
    rand_np = np.random.rand(10, 10).astype(np.float32)

    if torch.cuda.is_available():
        rand_torch = CoverageEnvUtils.to_tensor(rand_np).to("cuda")
        assert isinstance(rand_torch, torch.Tensor)
        assert rand_torch.shape == (10, 10)
        assert rand_torch.dtype == torch.float32
        is_all_close = np.allclose(rand_np, rand_torch.cpu().numpy())
        assert is_all_close
        is_all_equal = np.equal(rand_np, rand_torch.cpu().numpy()).all()

        if not is_all_equal and is_all_close:
            warnings.warn("Not all elements are equal, but all elements are close")
    else:
        warnings.warn("CUDA not available, skipping test_to_tensor_device")


def test_get_raw_local_maps():
    local_maps = CoverageEnvUtils.get_raw_local_maps(env, params)

    assert isinstance(local_maps, torch.Tensor)
    assert local_maps.shape == (
            params.pNumRobots,
            params.pLocalMapSize,
            params.pLocalMapSize,
            )
    assert local_maps.dtype == torch.float32
    saved_local_maps = torch.load(
            os.path.join(script_dir, "data/coverage_env_utils/local_maps.pt"),
            weights_only=True
            )
    is_all_close = torch.allclose(local_maps, saved_local_maps)
    assert is_all_close
    is_all_equal = torch.equal(local_maps, saved_local_maps)

    if not is_all_equal and is_all_close:
        warnings.warn("Not all elements are equal, but all elements are close")


def test_get_raw_obstacle_maps():
    obstacle_maps = CoverageEnvUtils.get_raw_obstacle_maps(env, params)
    assert isinstance(obstacle_maps, torch.Tensor)
    assert obstacle_maps.shape == (
            params.pNumRobots,
            params.pLocalMapSize,
            params.pLocalMapSize,
            )
    assert obstacle_maps.dtype == torch.float32
    saved_obstacle_maps = torch.load(
            os.path.join(script_dir, "data/coverage_env_utils/obstacle_maps.pt"),
            weights_only=True
            )
    is_all_close = torch.allclose(obstacle_maps, saved_obstacle_maps)
    assert is_all_close
    is_all_equal = torch.equal(obstacle_maps, saved_obstacle_maps)

    if not is_all_equal and is_all_close:
        warnings.warn("Not all elements are equal, but all elements are close")


def test_get_communication_maps():
    comm_maps = CoverageEnvUtils.get_communication_maps(env, params, 32)
    assert isinstance(comm_maps, torch.Tensor)
    assert comm_maps.shape == (params.pNumRobots, 2, 32, 32)
    assert comm_maps.dtype == torch.float32
    saved_comm_maps = torch.load(
            os.path.join(script_dir, "data/coverage_env_utils/comm_maps.pt"),
            weights_only=True
            )
    is_all_close = torch.allclose(comm_maps, saved_comm_maps)
    max_error = torch.max(torch.abs(comm_maps - saved_comm_maps))
    print(f"Max error: {max_error}")
    assert is_all_close
    is_all_equal = torch.equal(comm_maps, saved_comm_maps)

    if not is_all_equal and is_all_close:
        warnings.warn("Not all elements are equal, but all elements are close")


def test_resize_maps():
    local_maps = CoverageEnvUtils.get_raw_local_maps(env, params)
    resized_local_maps = CoverageEnvUtils.resize_maps(local_maps, 32)
    assert isinstance(resized_local_maps, torch.Tensor)
    assert resized_local_maps.shape == (params.pNumRobots, 32, 32)
    assert resized_local_maps.dtype == torch.float32
    saved_resized_local_maps = torch.load(
            os.path.join(script_dir, "data/coverage_env_utils/resized_local_maps.pt"),
            weights_only=True
            )
    is_all_close = torch.allclose(resized_local_maps, saved_resized_local_maps)
    assert is_all_close
    is_all_equal = torch.equal(resized_local_maps, saved_resized_local_maps)

    if not is_all_equal and is_all_close:
        warnings.warn("Not all elements are equal, but all elements are close")


def test_get_maps():
    maps = CoverageEnvUtils.get_maps(env, params, 32, use_comm_map=True)
    assert isinstance(maps, torch.Tensor)
    assert maps.shape == (params.pNumRobots, 4, 32, 32)
    assert maps.dtype == torch.float32
    saved_maps = torch.load(os.path.join(script_dir, "data/coverage_env_utils/maps.pt"), weights_only=True)
    is_all_close = torch.allclose(maps, saved_maps)
    assert is_all_close
    is_all_equal = torch.equal(maps, saved_maps)

    if not is_all_equal and is_all_close:
        warnings.warn("Not all elements are equal, but all elements are close")


def test_get_voronoi_features():
    voronoi_features = CoverageEnvUtils.get_voronoi_features(env)
    assert isinstance(voronoi_features, torch.Tensor)
    feature_len = len(env.GetRobotVoronoiFeatures()[0])
    assert voronoi_features.shape == (params.pNumRobots, feature_len)
    assert voronoi_features.dtype == torch.float32
    saved_voronoi_features = torch.load(
            os.path.join(script_dir, "data/coverage_env_utils/voronoi_features.pt"),
            weights_only=True
            )
    is_all_close = torch.allclose(voronoi_features, saved_voronoi_features)
    assert is_all_close
    is_all_equal = torch.equal(voronoi_features, saved_voronoi_features)

    if not is_all_equal and is_all_close:
        warnings.warn("Not all elements are equal, but all elements are close")


def test_get_robot_positions():
    robot_positions = CoverageEnvUtils.get_robot_positions(env)
    assert isinstance(robot_positions, torch.Tensor)
    assert robot_positions.shape == (params.pNumRobots, 2)
    assert robot_positions.dtype == torch.float32
    saved_robot_positions = torch.load(
            os.path.join(script_dir, "data/coverage_env_utils/robot_positions.pt"),
            weights_only=True
            )
    is_all_close = torch.allclose(robot_positions, saved_robot_positions)
    assert is_all_close
    is_all_equal = torch.equal(robot_positions, saved_robot_positions)

    if not is_all_equal and is_all_close:
        warnings.warn("Not all elements are equal, but all elements are close")


def test_get_weights():
    weights = CoverageEnvUtils.get_weights(env, params)
    assert isinstance(weights, torch.Tensor)
    assert weights.shape == (params.pNumRobots, params.pNumRobots)
    assert weights.dtype == torch.float32
    saved_weights = torch.load(
            os.path.join(script_dir, "data/coverage_env_utils/weights.pt"),
            weights_only=True
            )
    is_all_close = torch.allclose(weights, saved_weights)
    assert is_all_close
    is_all_equal = torch.equal(weights, saved_weights)

    if not is_all_equal and is_all_close:
        warnings.warn("Not all elements are equal, but all elements are close")


def test_get_torch_geometric_data():
    data = CoverageEnvUtils.get_torch_geometric_data(
            env, params, use_cnn=True, use_comm_map=True, map_size=32
            )
    assert isinstance(data, torch_geometric.data.Data)
    assert data.x.shape == (params.pNumRobots, 4, 32, 32)
    assert data.x.dtype == torch.float32
    assert data.edge_index.shape == (2, 16)
    assert data.edge_index.dtype == torch.long
    saved_data = torch_geometric.data.data.Data.from_dict(torch.load(
        os.path.join(script_dir, "data/coverage_env_utils/torch_geometric_data.pt"),
        weights_only=True
        ))
    is_all_close = torch.allclose(data.x, saved_data.x)
    assert is_all_close
    is_all_equal = torch.equal(data.x, saved_data.x)

    if not is_all_equal and is_all_close:
        warnings.warn("Not all elements are equal, but all elements are close")
    # assert data.x.equal(saved_data.x)

    is_all_close = torch.allclose(data.edge_index, saved_data.edge_index)
    assert is_all_close
    is_all_equal = torch.equal(data.edge_index, saved_data.edge_index)

    if not is_all_equal and is_all_close:
        warnings.warn("Not all elements are equal, but all elements are close")
    # assert data.edge_index.equal(saved_data.edge_index)
    # assert data.edge_index.equal(torch.load(os.path.join(script_dir, "data/coverage_env_utils/weights.pt")).to_sparse()._indices())
