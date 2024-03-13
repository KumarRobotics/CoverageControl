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
import numpy as np
import warnings
import torch
import torch_geometric

import coverage_control as cc
import coverage_control.nn as cc_nn
from coverage_control import IOUtils
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

torch.no_grad()
model_file = os.path.join(script_dir, "data/lpac/models/model_k3_1024_state_dict.pt")
learning_config_file = os.path.join(script_dir, "data/lpac/models/learning_params.toml")
learning_config = IOUtils.load_toml(learning_config_file)
lpac_model = cc_nn.LPAC(learning_config).to(device)
lpac_model.load_state_dict(torch.load(model_file))

lpac_model.eval()

use_comm_maps = learning_config['ModelConfig']['UseCommMaps']
map_size = learning_config['CNNBackBone']['ImageSize']

lpac_inputs = []
robot_pos_all = np.load(os.path.join(script_dir, "data/nn/robots_positions.npy"))
for i in range(0, 30):
    robot_pos = robot_pos_all[i]
    for j in range(0, params.pNumRobots):
        env.SetGlobalRobotPosition(j, robot_pos[j])
    pyg_data = CoverageEnvUtils.get_torch_geometric_data(env, params, True, use_comm_maps, map_size).to(device)
    lpac_inputs.append(pyg_data)

def test_cnn():
    with torch.no_grad():
        ref_cnn_outputs = torch.load(os.path.join(script_dir, "data/nn/cnn_outputs.pt"))
        cnn_model = lpac_model.cnn_backbone.to(device).eval()
        for i in range(0, len(lpac_inputs)):
            cnn_output = cnn_model(lpac_inputs[i].x)
            ref_cnn_outputs[i] = cnn_model(lpac_inputs[i].x)
            is_equal = torch.equal(cnn_output, ref_cnn_outputs[i])
            print(f"Output: at {i}")
            if not is_equal:
                error = torch.sum(torch.abs(cnn_output - ref_cnn_outputs[i]))
                print(f"Error: {error} at {i}")
                assert is_equal
                break

def test_lpac():
    with torch.no_grad():
        ref_lpac_outputs = torch.load(os.path.join(script_dir, "data/nn/lpac_outputs.pt"))
        for i in range(0, len(lpac_inputs)):
            lpac_output = lpac_model(lpac_inputs[i])
            ref_lpac_outputs[i] = lpac_output
            is_equal = torch.equal(lpac_output, ref_lpac_outputs[i])
            print(f"Output: at {i}")
            if not is_equal:
                error = torch.sum(torch.abs(lpac_output - ref_lpac_outputs[i]))
                print(f"Error: {error} at {i}")
                assert is_equal
                break

if __name__ == "__main__":
    test_cnn()
    test_lpac()
