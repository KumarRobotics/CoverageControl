# @file eval.py
#  @brief Evaluates the performance of the controllers on a set of environments
import os
import sys

import coverage_control as cc
import numpy as np
from coverage_control import CoverageSystem
from coverage_control import IOUtils
from coverage_control import WorldIDF
from coverage_control.algorithms import ControllerCVT
from coverage_control.algorithms import ControllerNN


# @ingroup python_api
class Evaluator:
    """
    Evaluates the performance of the controllers on a set of environments
    """

    def __init__(self, in_config):
        self.config = in_config
        self.eval_dir = IOUtils.sanitize_path(self.config["EvalDir"])
        self.env_dir = IOUtils.sanitize_path(self.config["EnvironmentDataDir"])
        self.controller_dir = None

        if not os.path.exists(self.env_dir):
            os.makedirs(self.env_dir)

        self.num_controllers = len(self.config["Controllers"])
        self.controllers_configs = self.config["Controllers"]

        for controller_config in self.controllers_configs:
            c_dir = self.eval_dir + "/" + controller_config["Name"]

            if not os.path.exists(c_dir):
                os.makedirs(c_dir)

        self.env_config_file = IOUtils.sanitize_path(self.config["EnvironmentConfig"])
        self.env_config = IOUtils.load_toml(self.env_config_file)
        self.cc_params = cc.Parameters(self.env_config_file)

        self.num_robots = self.cc_params.pNumRobots
        self.num_features = self.cc_params.pNumFeatures
        self.num_envs = self.config["NumEnvironments"]
        self.num_steps = self.config["NumSteps"]

    def evaluate(self, save=True):
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
                env_main = CoverageSystem(
                    self.cc_params, self.num_features, self.num_robots
                )
                env_main.WriteEnvironment(pos_file, env_file)
                world_idf = env_main.GetWorldIDFObject()

            robot_init_pos = env_main.GetRobotPositions(force_no_noise=True)

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
                controller = Controller(
                    self.controllers_configs[controller_id], self.cc_params, env
                )
                initial_objective_value = env.GetObjectiveValue()
                cost_data[controller_id, dataset_count, step_count] = (
                    env.GetObjectiveValue() / initial_objective_value
                )
                step_count = step_count + 1

                while step_count < self.num_steps:
                    objective_value, converged = controller.step(env)
                    cost_data[controller_id, dataset_count, step_count] = (
                        objective_value / initial_objective_value
                    )

                    if converged:
                        cost_data[controller_id, dataset_count, step_count:] = (
                            objective_value / initial_objective_value
                        )

                        break
                    # env.PlotMapVoronoi(map_dir, step_count)
                    # env.RecordPlotData()
                    step_count = step_count + 1

                    if step_count % 100 == 0:
                        val = cost_data[controller_id, dataset_count, step_count - 1]
                        print(
                            f"Environment {dataset_count} "
                            f"{controller.name} "
                            f"Step {step_count} "
                            f"Objective Value {val}"
                        )

                if save is True:
                    self.controller_dir = (
                        self.eval_dir
                        + "/"
                        + self.controllers_configs[controller_id]["Name"]
                    )
                    controller_data_file = self.controller_dir + "/" + "eval.csv"
                    np.savetxt(
                        controller_data_file,
                        cost_data[controller_id, : dataset_count + 1, :],
                        delimiter=",",
                    )
                # env.RenderRecordedMap(self.eval_dir + "/" + self.controllers[controller_id]["Name"] + "/", "video.mp4")
                del controller
                del env
            dataset_count = dataset_count + 1

        return cost_data


if __name__ == "__main__":
    config_file = sys.argv[1]
    config = IOUtils.load_toml(config_file)

    evaluator = Evaluator(config)
    evaluator.evaluate()
