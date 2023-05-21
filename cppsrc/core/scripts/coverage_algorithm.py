import sys
import os
import math
import time
import numpy as np
import pyCoverageControl # Main library
from pyCoverageControl import Point2 # for defining points
from pyCoverageControl import PointVector # for defining list of points
from pyCoverageControl import CoverageSystem
# Algorithms available:
# LloydGlobalOnline
# LloydLocalVornoi
# OracleGlobalOffline
# OracleSimulExploreExploit
from pyCoverageControl import OracleSimulExploreExploit as CoverageAlgorithm

class RunCoverageAlgorithm:

    def __init__(self, params_filename='parameters.yaml', num_gaussians=5, num_robots=15, num_steps_per_dataset=1, stable_dataset_count=90):
        self.params_ = pyCoverageControl.Parameters(params_filename)
        self.num_gaussians = num_gaussians
        self.num_robots = num_robots
        self.num_steps_per_dataset = num_steps_per_dataset
        self.stable_dataset_count = stable_dataset_count

        self.env = CoverageSystem(self.params_, self.num_gaussians, self.num_robots)
        self.oracle = CoverageAlgorithm(self.params_, self.num_robots, self.env)

    def Step(self):
        cont_flag = self.oracle.Step();
        actions = self.oracle.GetActions()
        error_flag = self.env.StepActions(actions)
        return cont_flag, error_flag

    def Execute(self):
        num_steps = 0
        cont_flag = True

        while num_steps < math.floor(self.params_.pEpisodeSteps):
            for i in range(0, self.num_steps_per_dataset - 1):
                [cont_flag, error_flag] = self.Step()
                num_steps = num_steps + 1
                if cont_flag == False:
                    break
            if cont_flag == False:
                break

            [cont_flag, error_flag] = self.Step()
            num_steps = num_steps + 1
            self.env.RecordPlotData()
            print(str(num_steps) + " steps completed")

        zero_actions = self.oracle.GetActions()
        for action in zero_actions:
            action = Point2(0, 0)

        print("Exploration ratio: " + str(self.env.GetExplorationRatio()) + " Weighted exploration ratio: " + str(self.env.GetWeightedExplorationRatio()))
        print("Coverage objective: " + str('{:.2e}'.format(self.env.GetObjectiveValue())))
        for i in range(0, self.stable_dataset_count):
            error_flag = self.env.StepActions(zero_actions)
            self.env.RecordPlotData()

if __name__ == '__main__':
    params_filename = "params/parameters.yaml"
    dir = "."

    num_gaussians = 5
    num_robots = 15

    gen = RunCoverageAlgorithm(params_filename, num_gaussians, num_robots)
    gen.Execute()
    gen.env.RenderRecordedMap(dir, "video.mp4")
