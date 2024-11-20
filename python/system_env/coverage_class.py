"""
An example class to use the CoverageControl library to run a coverage algorithm
"""

import sys

import coverage_control as cc  # Main library
from coverage_control import CoverageSystem
from coverage_control.algorithms import ClairvoyantCVT as CoverageAlgorithm
# Algorithms available:
# ClairvoyantCVT
# CentralizedCVT
# DecentralizedCVT
# NearOptimalCVT


class RunCoverageAlgorithm:
    """
    A class to run the coverage algorithm
    """

    def __init__(self, params_filename=None):
        if params_filename is not None:
            self.params_ = cc.Parameters(params_filename)
        else:
            self.params_ = cc.Parameters()

        self.params_.pNumGaussianFeatures = 5
        self.params_.pMaxSigma = 100
        self.params_.pMinSigma = 100


        self.env = CoverageSystem(self.params_)
        self.env.PlotInitMap('Init')
        self.controller = CoverageAlgorithm(
            self.params_, self.params_.pNumRobots, self.env
        )

    def step(self):
        """
        Run one step of the coverage algorithm
        """
        self.controller.ComputeActions()
        actions = self.controller.GetActions()
        error_flag = self.env.StepActions(actions)

        return error_flag

    def execute(self):
        """
        Run the coverage algorithm
        """
        num_steps = 1

        init_cost = self.env.GetObjectiveValue()
        print(f"Initial Coverage cost: {init_cost:.2e}")

        while num_steps <= self.params_.pEpisodeSteps:
            if self.step():
                print(f"Error in step {num_steps}")

                break

            if self.controller.IsConverged():
                print(f"Converged in step {num_steps}")

                break

            num_steps = num_steps + 1

        final_cost = self.env.GetObjectiveValue()
        print(f"Improvement %: {100 * (init_cost - final_cost)/init_cost:.2f}")
        self.env.PlotSystemMap('Final')


if __name__ == "__main__":
    if len(sys.argv) > 1:
        cc = RunCoverageAlgorithm(sys.argv[1])
    else:
        cc = RunCoverageAlgorithm()
    cc.execute()
