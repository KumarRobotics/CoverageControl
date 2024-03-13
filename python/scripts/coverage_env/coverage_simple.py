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

import sys
import coverage_control as cc
# Algorithms available:
# ClairvoyantCVT
# CentralizedCVT
# DecentralizedCVT
# NearOptimalCVT
from coverage_control.algorithms import ClairvoyantCVT as CoverageAlgorithm

params = cc.Parameters()

# CoverageSystem handles the environment and robots
env = cc.CoverageSystem(params)

init_cost = env.GetObjectiveValue()
print("Initial Coverage cost: " + str('{:.2e}'.format(init_cost)))

# Runs the coverage control algorithm
controller = CoverageAlgorithm(params, env)

for i in range(0, params.pEpisodeSteps):
    # Compute actions to be taken by the robots
    controller.ComputeActions();
    # Get actions from the controller
    actions = controller.GetActions()
    # Send actions to the environment
    if env.StepActions(actions):
        print("Error in step " + str(i))
        break

    if controller.IsConverged():
        print("Converged in step " + str(i))
        break

# print some metrics
current_cost = env.GetObjectiveValue()
print("Improvement %: " + str('{:.2f}'.format(100 * (init_cost - current_cost)/init_cost)))
