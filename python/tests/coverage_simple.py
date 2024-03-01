import sys
import CoverageControl # Main library
# Algorithms available:
# ClairvoyantCVT
# CentralizedCVT
# DecentralizedCVT
# NearOptimalCVT
from CoverageControl import ClairvoyantCVT as CoverageAlgorithm

params = CoverageControl.Parameters()

# CoverageSystem handles the environment and robots
env = CoverageControl.CoverageSystem(params)

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
