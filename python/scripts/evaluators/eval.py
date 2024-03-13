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

class Evaluator:
    def __init__(self, config):
        self.config = config
        self.eval_dir = IOUtils.sanitize_path(self.config["EvalDir"])
        self.env_dir = IOUtils.sanitize_path(self.config["EnvironmentDataDir"])
        if not os.path.exists(self.env_dir):
            os.makedirs(self.env_dir)

        self.num_controllers = len(self.config["Controllers"])
        self.controllers_configs = self.config["Controllers"]

        for controller_config in self.controllers_configs:
            controller_dir = self.eval_dir + "/" + controller_config["Name"]
            if not os.path.exists(controller_dir):
                os.makedirs(controller_dir)

        self.env_config_file = IOUtils.sanitize_path(self.config["EnvironmentConfig"])
        self.env_config = IOUtils.load_toml(self.env_config_file)
        self.cc_params = cc.Parameters(self.env_config_file)

        self.num_robots = self.cc_params.pNumRobots
        self.num_features = self.cc_params.pNumFeatures
        self.num_envs = self.config["NumEnvironments"]
        self.num_steps = self.config["NumSteps"]

    def Evaluate(self, save = True):
        dataset_count = 0
        cost_data = np.zeros((self.num_controllers, self.num_envs, self.num_steps))
        while dataset_count < self.num_envs:
            print(f"Environment {dataset_count}")
            pos_file = self.env_dir + "/" + str(dataset_count) + ".pos"
            env_file = self.env_dir + "/" + str(dataset_count) + ".env"
            if os.path.isfile(env_file) and os.path.isfile(pos_file):
                world_idf = WorldIDF(self.cc_params, env_file)
                env_main = CoverageSystem(self.cc_params, world_idf, pos_file)
            else:
                print(f"Creating new environment {dataset_count}")
                env_main = CoverageSystem(self.cc_params, self.num_features, self.num_robots)
                env_main.WriteEnvironment(pos_file, env_file)
                world_idf = env_main.GetWorldIDFObject()

            robot_init_pos = env_main.GetRobotPositions(force_no_noise = True)
            for controller_id in range(self.num_controllers):
                step_count = 0
                env = CoverageSystem(self.cc_params, world_idf, robot_init_pos)

                # map_dir = self.eval_dir + "/" + self.controllers[controller_id]["Name"] + "/plots/"
                # os.makedirs(map_dir, exist_ok = True)
                # env.PlotInitMap(map_dir, "InitMap")
                # env.RecordPlotData()
                # env.PlotMapVoronoi(map_dir, step_count)

                if self.controllers_configs[controller_id]["Type"] == "Learning":
                    Controller = ControllerNN
                else:
                    Controller = ControllerCVT
                controller = Controller(self.controllers_configs[controller_id], self.cc_params, env)
                cost_data[controller_id, dataset_count, step_count] = env.GetObjectiveValue()
                step_count = step_count + 1
                while step_count < self.num_steps:
                    objective_value, converged = controller.Step(env)
                    cost_data[controller_id, dataset_count, step_count] = objective_value
                    if converged:
                        cost_data[controller_id, dataset_count, step_count:] = objective_value
                        break
                    # env.PlotMapVoronoi(map_dir, step_count)
                    # env.RecordPlotData()
                    step_count = step_count + 1
                    if step_count % 100 == 0:
                        print(f"Step {step_count}, Objective Value {objective_value}")
                        print(f"Environment {dataset_count}, {controller.name}, Step {step_count}")

                if save == True:
                    self.controller_dir = self.eval_dir + "/" + self.controllers_configs[controller_id]["Name"]
                    controller_data_file = self.controller_dir + "/" + "eval.csv"
                    np.savetxt(controller_data_file, cost_data[controller_id, :dataset_count + 1, :], delimiter=",")
                # env.RenderRecordedMap(self.eval_dir + "/" + self.controllers[controller_id]["Name"] + "/", "video.mp4")
                del controller
                del env
            dataset_count = dataset_count + 1
        return cost_data


if __name__ == "__main__":

    config_file = sys.argv[1]
    config = IOUtils.load_toml(config_file)

    evaluator = Evaluator(config)
    evaluator.Evaluate()
