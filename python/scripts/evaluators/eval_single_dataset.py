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

import os
import sys
import numpy as np

import torch

import coverage_control as cc
from coverage_control import IOUtils
from coverage_control import CoverageSystem
from coverage_control import Parameters, WorldIDF
from controller import ControllerCVT, ControllerNN

class EvaluatorSingle:
    def __init__(self, config):
        self.config = config
        self.eval_dir = IOUtils.sanitize_path(self.config["EvalDir"]) + "/"
        self.env_dir = IOUtils.sanitize_path(self.config["EnvironmentDataDir"]) + "/"
        self.feature_file = self.env_dir + self.config["FeatureFile"]
        self.pos_file = self.env_dir + self.config["RobotPosFile"]
        if not os.path.exists(self.env_dir):
            os.makedirs(self.env_dir)

        if not os.path.exists(self.feature_file):
            raise ValueError(f"Feature file {self.feature_file} does not exist")
        if not os.path.exists(self.pos_file):
            raise ValueError(f"Robot position file {self.pos_file} does not exist")

        self.num_controllers = len(self.config["Controllers"])
        self.controllers_configs = self.config["Controllers"]

        self.env_config_file = IOUtils.sanitize_path(self.config["EnvironmentConfig"])
        self.env_config = IOUtils.load_toml(self.env_config_file)
        self.cc_params = cc.Parameters(self.env_config_file)

        self.num_robots = self.cc_params.pNumRobots
        self.num_features = self.cc_params.pNumFeatures
        self.num_steps = self.config["NumSteps"]

        self.plot_map = self.config["PlotMap"]
        self.generate_video = self.config["GenerateVideo"]

    def Evaluate(self, save = True):
        cost_data = np.zeros((self.num_controllers, self.num_steps))
        world_idf = WorldIDF(self.cc_params, self.feature_file)
        env_main = CoverageSystem(self.cc_params, world_idf, self.pos_file)
        robot_init_pos = env_main.GetRobotPositions(force_no_noise = True)

        if self.plot_map:
            map_dir = self.eval_dir + "/plots/"
            os.makedirs(map_dir, exist_ok = True)
            env_main.PlotInitMap(map_dir, "InitMap")

        for controller_id in range(self.num_controllers):
            step_count = 0
            env = CoverageSystem(self.cc_params, world_idf, robot_init_pos)
            controller_name = self.controllers_configs[controller_id]["Name"]

            if self.generate_video:
                env.RecordPlotData()

            if self.controllers_configs[controller_id]["Type"] == "Learning":
                Controller = ControllerNN
            else:
                Controller = ControllerCVT
            controller = Controller(self.controllers_configs[controller_id], self.cc_params, env)
            cost_data[controller_id, step_count] = env.GetObjectiveValue()
            step_count = step_count + 1
            while step_count < self.num_steps:
                objective_value, converged = controller.Step(env)
                cost_data[controller_id, step_count] = objective_value
                if converged and not self.generate_video:
                    cost_data[controller_id, step_count:] = objective_value
                    break
                if self.generate_video:
                    env.RecordPlotData()

                step_count = step_count + 1
                if step_count % 100 == 0:
                    print(f"Step {step_count}, Objective Value {objective_value}")
                    print(f"{controller.name}, Step {step_count}")

            if save == True:
                controller_dir = self.eval_dir + "/" + controller_name
                if not os.path.exists(controller_dir):
                    os.makedirs(controller_dir)

                controller_data_file = controller_dir + "/" + "eval.csv"
                np.savetxt(controller_data_file, cost_data[controller_id, :], delimiter=",")
            if self.generate_video:
                controller_dir = self.eval_dir + "/" + controller_name
                env.RenderRecordedMap(controller_dir, "video.mp4")

            del controller
            del env
        return cost_data


if __name__ == "__main__":

    config_file = sys.argv[1]
    config = IOUtils.load_toml(config_file)

    evaluator = EvaluatorSingle(config)
    evaluator.Evaluate()
