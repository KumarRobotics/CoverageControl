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

import tempfile
import numpy as np
import coverage_control as cc

def test_env_io():
    params = cc.Parameters()
    env = cc.CoverageSystem(params)
    world_map = env.GetWorldMap()

    with tempfile.TemporaryDirectory() as tmp_dir:
        env.WriteEnvironment(tmp_dir + "/env.pos", tmp_dir + "/env.idf")
        world_idf = cc.WorldIDF(params, tmp_dir + "/env.idf")
        env2 = cc.CoverageSystem(params, world_idf, tmp_dir + "/env.pos")
        map2 = env2.GetWorldMap()
    is_equal = np.array_equal(world_map, map2)
    assert is_equal


