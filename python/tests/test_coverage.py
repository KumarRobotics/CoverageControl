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

import coverage_control as cc

# Algorithms available:
# ClairvoyantCVT
# CentralizedCVT
# DecentralizedCVT
# NearOptimalCVT (not deterministic and not tested)
from coverage_control.algorithms import CentralizedCVT, ClairvoyantCVT, DecentralizedCVT

params = cc.Parameters()
params.pNumGaussianFeatures = 3
params.pNumRobots = 3

bnd_list = cc.BNDVector()
bnd_list.append(cc.BivariateNormalDistribution(cc.Point2(182, 438), 55, 8))
bnd_list.append(cc.BivariateNormalDistribution(cc.Point2(950, 97), 47, 7))
bnd_list.append(cc.BivariateNormalDistribution(cc.Point2(714, 920), 46, 7))

robot_pos_list = cc.PointVector()
robot_pos_list.append(cc.Point2(433, 154))
robot_pos_list.append(cc.Point2(139, 319))
robot_pos_list.append(cc.Point2(92, 112))


def test_clairvoyant_cvt():
    env = cc.CoverageSystem(params, bnd_list, robot_pos_list)
    init_cost = env.GetObjectiveValue()
    controller = ClairvoyantCVT(params, env)

    for i in range(0, params.pEpisodeSteps):
        controller.ComputeActions()
        actions = controller.GetActions()

        if env.StepActions(actions):
            print("Error in step " + str(i))

            break

        if controller.IsConverged():
            break
        step_num = i
    current_cost = env.GetObjectiveValue()
    improvement = 100 * (init_cost - current_cost) / init_cost
    improvement_rounded = round(improvement, 2)
    assert improvement_rounded == 98.90
    assert step_num == 166


def test_centralized_cvt():
    env = cc.CoverageSystem(params, bnd_list, robot_pos_list)
    init_cost = env.GetObjectiveValue()
    controller = CentralizedCVT(params, env)

    for i in range(0, params.pEpisodeSteps):
        controller.ComputeActions()
        actions = controller.GetActions()

        if env.StepActions(actions):
            print("Error in step " + str(i))

            break

        if controller.IsConverged():
            break
        step_num = i
    current_cost = env.GetObjectiveValue()
    improvement = 100 * (init_cost - current_cost) / init_cost
    improvement_rounded = round(improvement, 2)
    assert improvement_rounded == 58.13
    assert step_num == 104


def test_decentralized_cvt():
    env = cc.CoverageSystem(params, bnd_list, robot_pos_list)
    init_cost = env.GetObjectiveValue()
    controller = DecentralizedCVT(params, env)

    for i in range(0, params.pEpisodeSteps):
        controller.ComputeActions()
        actions = controller.GetActions()

        if env.StepActions(actions):
            print("Error in step " + str(i))

            break

        if controller.IsConverged():
            break
        step_num = i
    current_cost = env.GetObjectiveValue()
    improvement = 100 * (init_cost - current_cost) / init_cost
    improvement_rounded = round(improvement, 2)
    assert improvement_rounded == 8.87
    assert step_num == 17


if __name__ == "__main__":
    test_clairvoyant_cvt()
    test_centralized_cvt()
    test_decentralized_cvt()
