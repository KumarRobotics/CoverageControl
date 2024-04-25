"""
A simple example of using the CoverageControl library
"""

import coverage_control as cc
from coverage_control.algorithms import ClairvoyantCVT as CoverageAlgorithm
# Algorithms available:
# ClairvoyantCVT
# CentralizedCVT
# DecentralizedCVT
# NearOptimalCVT

params = cc.Parameters()

# CoverageSystem handles the environment and robots
env = cc.CoverageSystem(params)

init_cost = env.GetObjectiveValue()
print(f"Initial Coverage cost: {init_cost:.2e}")

# Runs the coverage control algorithm
controller = CoverageAlgorithm(params, env)

for i in range(0, params.pEpisodeSteps):
    # Compute actions to be taken by the robots
    controller.ComputeActions()
    # Get actions from the controller
    actions = controller.GetActions()

    # Send actions to the environment

    if env.StepActions(actions):
        print(f"Error in step {i}")

        break

    if controller.IsConverged():
        print(f"Converged in step {i}")

        break

# print some metrics
current_cost = env.GetObjectiveValue()
print(f"Improvement %: {100 * (init_cost - current_cost)/init_cost:.2f}")
