import sys
import CoverageControl # Main library
from CoverageControl import CoverageSystem

# Algorithms available:
# ClairvoyantCVT
# CentralizedCVT
# DecentralizedCVT
# NearOptimalCVT
from CoverageControl import ClairvoyantCVT as CoverageAlgorithm

class RunCoverageAlgorithm:

    def __init__(self, params_filename=None):
        if params_filename is not None:
            self.params_ = CoverageControl.Parameters(params_filename)
        else:
            self.params_ = CoverageControl.Parameters()

        self.env = CoverageSystem(self.params_)
        self.controller = CoverageAlgorithm(self.params_, self.params_.pNumRobots, self.env)

    def Step(self):
        self.controller.ComputeActions();
        actions = self.controller.GetActions()
        error_flag = self.env.StepActions(actions)
        return error_flag

    def Execute(self):
        num_steps = 1

        init_cost = self.env.GetObjectiveValue()
        print("Initial Coverage cost: " + str('{:.2e}'.format(init_cost)))

        while num_steps <= self.params_.pEpisodeSteps:
            if self.Step():
                print("Error in step " + str(num_steps))
                break

            if self.controller.IsConverged():
                print("Converged in step " + str(num_steps))
                break

            num_steps = num_steps + 1

        final_cost = self.env.GetObjectiveValue()
        print("Improvement %: " + str('{:.2f}'.format(100 * (init_cost - final_cost)/init_cost)))

if __name__ == '__main__':

    if len(sys.argv) > 1:
        cc = RunCoverageAlgorithm(sys.argv[1])
    else:
        cc = RunCoverageAlgorithm()
    cc.Execute()
