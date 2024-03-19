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
import sys

if sys.version_info[1] < 11:
    import tomli as tomllib
else:
    import tomllib

import coverage_control as cc


def test_parameters():
    # Test the parameters
    script_dir = os.path.dirname(os.path.realpath(__file__))
    params_file = os.path.join(script_dir, "data/params/coverage_control_params.toml")
    params = cc.Parameters(params_file)

    with open(params_file, "rb") as f:
        params_toml = tomllib.load(f)

    dir_params = params.__dir__()

    # Remove the private variables
    dir_params = [x for x in dir_params if "__" not in x]

    # Remove functions and strip the p
    dir_params = [x[1:] for x in dir_params if "p" in x]

    # Extract all the leaf nodes from the toml file
    def extract_leaf_nodes(params_toml):
        leaf_nodes = []

        for key in params_toml:
            if isinstance(params_toml[key], dict):
                leaf_nodes.extend(extract_leaf_nodes(params_toml[key]))
            else:
                leaf_nodes.append(key)

        return leaf_nodes

    params_toml = extract_leaf_nodes(params_toml)

    # Check if all the parameters are present
    # for key in params_toml:
    #     assert key in dir_params

    for key in dir_params:
        assert key in params_toml


if __name__ == "__main__":
    test_parameters()
