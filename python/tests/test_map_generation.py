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

script_dir = os.path.dirname(os.path.realpath(__file__))

params_file = os.path.join(script_dir, "data/params/coverage_control_params.toml")
params = coverage_control.Parameters(params_file)

features_file = os.path.join(script_dir, "data/features")
robot_pos_file = os.path.join(script_dir, "data/robots_positions")


def test_map_generation():
    world_idf = coverage_control.WorldIDF(
        params, os.path.join(script_dir, features_file)
    )
    env = coverage_control.CoverageSystem(params, world_idf, robot_pos_file)
    world_map = env.GetWorldMap()

    world_map_ref = np.load(os.path.join(script_dir, "data/world_map.npy"))
    is_all_close = np.allclose(world_map, world_map_ref, atol=1e-2)
    assert is_all_close
    is_all_equal = np.equal(world_map, world_map_ref).all()

    if not is_all_equal and is_all_close:
        print("Max error: ", np.max(np.abs(world_map - world_map_ref)))
        warnings.warn("Not all elements are equal, but all elements are close")


if __name__ == "__main__":
    test_map_generation()
