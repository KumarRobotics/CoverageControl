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
import warnings

import coverage_control.nn as cc_nn
import torch
import torch_geometric
from coverage_control import IOUtils

script_dir = os.path.dirname(os.path.realpath(__file__))

device = torch.device("cpu")

with torch.no_grad():
    model_file = os.path.join(
            script_dir, "data/lpac/models/model_k3_1024_state_dict.pt"
            )
    learning_config_file = os.path.join(
            script_dir, "data/params/learning_params.toml"
            )
    learning_config = IOUtils.load_toml(learning_config_file)
    lpac_model = cc_nn.LPAC(learning_config).to(device)
    lpac_model.load_state_dict(torch.load(model_file, weights_only=True))

    lpac_model.eval()

    use_comm_maps = learning_config["ModelConfig"]["UseCommMaps"]
    map_size = learning_config["CNNBackBone"]["ImageSize"]

    lpac_inputs_dict = torch.load(os.path.join(script_dir, "data/lpac/lpac_inputs.pt"), weights_only=True)
    lpac_inputs = [torch_geometric.data.Data.from_dict(d) for d in lpac_inputs_dict]


def test_cnn():
    with torch.no_grad():
        ref_cnn_outputs = torch.load(
                os.path.join(script_dir, "data/lpac/cnn_outputs.pt"),
                weights_only=True
                )
        cnn_model = lpac_model.cnn_backbone.to(device).eval()

        for i in range(0, len(lpac_inputs)):
            cnn_output = cnn_model(lpac_inputs[i].x)
            is_close = torch.allclose(cnn_output, ref_cnn_outputs[i], atol=1e-4)

            if not is_close:
                error = torch.sum(torch.abs(cnn_output - ref_cnn_outputs[i]))
                print(f"Error: {error} at {i}")
                assert is_close

                break
            is_equal = torch.equal(cnn_output, ref_cnn_outputs[i])

            if not is_equal and is_close:
                error = torch.sum(torch.abs(cnn_output - ref_cnn_outputs[i]))
                print(f"Error: {error} at {i}")
                warnings.warn("Outputs are close but not equal")


def test_lpac():
    with torch.no_grad():
        ref_lpac_outputs = torch.load(
                os.path.join(script_dir, "data/lpac/lpac_outputs.pt"),
                weights_only=True
                )

        for i in range(0, len(lpac_inputs)):
            lpac_output = lpac_model(lpac_inputs[i])
            is_close = torch.allclose(lpac_output, ref_lpac_outputs[i], atol=1e-4)

            if not is_close:
                error = torch.sum(torch.abs(lpac_output - ref_lpac_outputs[i]))
                print(f"Error: {error} at {i}")
                assert is_close

                break
            is_equal = torch.equal(lpac_output, ref_lpac_outputs[i])

            if not is_equal and is_close:
                error = torch.sum(torch.abs(lpac_output - ref_lpac_outputs[i]))
                print(f"Error: {error} at {i}")
                warnings.warn("Outputs are close but not equal")


if __name__ == "__main__":
    test_cnn()
    test_lpac()
