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
A simple example of using WorldIDF
"""

import coverage_control
import numpy as np

params = coverage_control.Parameters()
params.pNumRobots = 1
params.pWorldMapSize = 5
initial_positions = coverage_control.PointVector()
initial_positions.append(coverage_control.Point2(0, 0))

##################################################
# Create a world IDF using a random numpy matrix #
##################################################

# Random numpy matrix
np_matrix = np.random.rand(params.pWorldMapSize, params.pWorldMapSize).astype(
    np.float32
)

world_idf = coverage_control.WorldIDF(params, np_matrix)
env = coverage_control.CoverageSystem(params, world_idf, initial_positions)

init_cost = env.GetObjectiveValue()
print(f"Initial Coverage cost: {init_cost:.3e}")

env_matrix = env.GetWorldMap()

assert np.allclose(np_matrix, env_matrix)
assert np.all(np_matrix == env_matrix)

print("")
print("Flags for the map: ", env_matrix.flags)
print("Env matrix: ", env_matrix)

###############################################
# Get the world IDF as a mutable numpy matrix #
###############################################
env_matrix_mutable = env.GetWorldMapMutable()

for i in range(env_matrix_mutable.shape[0]):
    for j in range(env_matrix_mutable.shape[1]):
        env_matrix_mutable[i, j] = i + j

env_matrix_new = env.GetWorldMap()

assert np.allclose(env_matrix_mutable, env_matrix_new)
assert np.all(env_matrix_mutable == env_matrix_new)

print("")
print("Flags for the map: ", env_matrix_new.flags)
print("Env matrix: ", env_matrix_new)
