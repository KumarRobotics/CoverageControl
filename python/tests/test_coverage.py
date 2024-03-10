from __future__ import annotations

import importlib.metadata
import test as m
from testcoverage import _core as cc # Main library
# Algorithms available:
# ClairvoyantCVT
# CentralizedCVT
# DecentralizedCVT
# NearOptimalCVT
from testcoverage._core import ClairvoyantCVT as CoverageAlgorithm

def test_coverage_system():
    params = cc.Parameters()
    params.pNumFeatures = 3
    params.pNumRobots = 3

    bnd_list = cc.BNDVector()
    bnd_list.append(cc.BivariateNormalDistribution(cc.Point2(182, 438), 55, 8))
    bnd_list.append(cc.BivariateNormalDistribution(cc.Point2(950, 97), 47, 7))
    bnd_list.append(cc.BivariateNormalDistribution(cc.Point2(714, 920), 46, 7))

    robot_pos_list = cc.PointVector()
    robot_pos_list.append(cc.Point2(433, 154))
    robot_pos_list.append(cc.Point2(139, 319))
    robot_pos_list.append(cc.Point2(92, 112))

    # CoverageSystem handles the environment and robots
    env = cc.CoverageSystem(params, bnd_list, robot_pos_list)

    init_cost = env.GetObjectiveValue()

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
        step_num = i

    # print some metrics
    current_cost = env.GetObjectiveValue()
    improvement = 100 * (init_cost - current_cost)/init_cost
    improvement_rounded = round(improvement, 2)
    assert improvement_rounded == 98.90
    assert step_num == 166
