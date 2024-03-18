# This file is part of the CoverageControl library
#
# Author: Saurav Agarwal
# Contact: sauravag@seas.upenn.edu, agr.saurav1@gmail.com
# Repository: https://github.com/KumarRobotics/CoverageControl
#
# Copyright (c) 2024, Saurav Agarwal
#
# The CoverageControl library is free software: you can redistribute it and/or
# modify it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or (at your
# option) any later version.
#
# The CoverageControl library is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
# Public License for more details.
#
# You should have received a copy of the GNU General Public License along with
# CoverageControl library. If not, see <https://www.gnu.org/licenses/>.

import os
import sys
import tempfile
import warnings
import torch

import coverage_control as cc
import coverage_control.nn as cc_nn
from coverage_control import IOUtils
from coverage_control import PointVector
from coverage_control.nn import CoverageEnvUtils

script_dir = os.path.dirname(os.path.realpath(__file__))

params_file = os.path.join(script_dir, "data/lpac/coverage_control_params.toml")
params = cc.Parameters(params_file)

features_file = os.path.join(script_dir, "data/nn/features")
robot_pos_file = os.path.join(script_dir, "data/nn/robots_positions")

world_idf = cc.WorldIDF(params, os.path.join(script_dir, features_file))
env = cc.CoverageSystem(params, world_idf, robot_pos_file)

params.pNumRobots = env.GetNumRobots()

device = torch.device('cpu')

with torch.no_grad():
    model_file = os.path.join(script_dir, "data/lpac/models/model_k3_1024_state_dict.pt")
    learning_config_file = os.path.join(script_dir, "data/lpac/models/learning_params.toml")
    learning_config = IOUtils.load_toml(learning_config_file)
    lpac_model = cc_nn.LPAC(learning_config).to(device)
    lpac_model.load_state_dict(torch.load(model_file))

    actions_mean = lpac_model.actions_mean.to(device)
    actions_std = lpac_model.actions_std.to(device)
    lpac_model.eval()

    use_comm_maps = learning_config['ModelConfig']['UseCommMaps']
    map_size = learning_config['CNNBackBone']['ImageSize']

def test_lpac_coverage():
    num_steps = 200
    step_count = 0

    actions_all = torch.zeros(num_steps, params.pNumRobots, 2)
    obj_values = torch.zeros(num_steps)
    init_obj_value = env.GetObjectiveValue()
    obj_values[0] = env.GetObjectiveValue()/init_obj_value

    step_count += 1
    while step_count < num_steps:
        with torch.no_grad():
            pyg_data = CoverageEnvUtils.get_torch_geometric_data(env, params, True, use_comm_maps, map_size).to(device)
            actions = lpac_model(pyg_data)
            actions = actions * actions_std + actions_mean
            actions_all[step_count] = actions
            point_vector_actions = PointVector(actions.cpu().numpy())
            env.StepActions(point_vector_actions)
            obj_values[step_count] = env.GetObjectiveValue()/init_obj_value
            step_count += 1
    lpac_actions_ref = torch.load(os.path.join(script_dir, "data/nn/lpac_actions.pt"))
    is_all_close = torch.allclose(actions_all, lpac_actions_ref, atol=1e-2)
    assert is_all_close
    is_all_equal = torch.equal(actions_all, lpac_actions_ref)
    if not is_all_equal and is_all_close:
        warnings.warn("Not all elements are equal, but all elements are close")

    lpac_obj_values_ref = torch.load(os.path.join(script_dir, "data/nn/lpac_obj_values.pt"))
    is_all_close = torch.allclose(obj_values, lpac_obj_values_ref, atol=1e-2)
    assert is_all_close
    is_all_equal = torch.equal(obj_values, lpac_obj_values_ref)
    if not is_all_equal and is_all_close:
        warnings.warn("Not all elements are equal, but all elements are close")

if __name__ == "__main__":
    test_lpac_coverage()
