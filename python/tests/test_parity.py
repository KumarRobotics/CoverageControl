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

import warnings

import coverage_control
import numpy as np

params = coverage_control.Parameters()


def test_parity():
    if not coverage_control.CudaUtils.IsCudaAvailable():
        warnings.warn("CUDA not available, skipping test")

        return
    env_cuda = coverage_control.CoverageSystem(params)
    world_idf_obj_cuda = env_cuda.GetWorldIDFObject()
    world_idf_obj_cuda.GenerateMapCuda()
    world_map_cuda = world_idf_obj_cuda.GetWorldMap()
    init_robots = env_cuda.GetRobotPositions()

    env_cpu = coverage_control.CoverageSystem(params, world_idf_obj_cuda, init_robots)
    world_idf_obj_cpu = env_cpu.GetWorldIDFObject()
    world_idf_obj_cpu.GenerateMapCPU()
    world_map_cpu = world_idf_obj_cpu.GetWorldMap()

    is_close = np.allclose(world_map_cuda, world_map_cpu, atol=1e-2)

    if not is_close:
        diff = np.abs(world_map_cuda - world_map_cpu).max()
        print("Max difference: ", diff)
    assert is_close
    is_equal = np.array_equal(world_map_cuda, world_map_cpu)

    if not is_equal and is_close:
        diff = np.abs(world_map_cuda - world_map_cpu).max()
        print("Max difference: ", diff)
        warnings.warn("Not all elements are equal, but all elements are close")


if __name__ == "__main__":
    test_parity()
