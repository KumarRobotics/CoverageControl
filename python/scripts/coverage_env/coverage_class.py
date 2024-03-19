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
An example class to use the CoverageControl library to run a coverage algorithm
"""
import sys
import coverage_control as cc # Main library
from coverage_control import CoverageSystem

# Algorithms available:
# ClairvoyantCVT
# CentralizedCVT
# DecentralizedCVT
# NearOptimalCVT
from coverage_control.algorithms import ClairvoyantCVT as CoverageAlgorithm

class RunCoverageAlgorithm:
    """
    A class to run the coverage algorithm
    """

    def __init__(self, params_filename=None):
        if params_filename is not None:
            self.params_ = cc.Parameters(params_filename)
        else:
            self.params_ = cc.Parameters()

        self.env = CoverageSystem(self.params_)
        self.controller = CoverageAlgorithm(self.params_, self.params_.pNumRobots, self.env)

    def step(self):
        """
        Run one step of the coverage algorithm
        """
        self.controller.ComputeActions();
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

if __name__ == '__main__':

    if len(sys.argv) > 1:
        cc = RunCoverageAlgorithm(sys.argv[1])
    else:
        cc = RunCoverageAlgorithm()
    cc.execute()
