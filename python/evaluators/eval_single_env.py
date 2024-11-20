# @file eval_single_env.py
#  @brief Evaluate a single dataset with multiple controllers
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
class EvaluatorSingle:
    """
    Class to evaluate a single environment with multiple controllers
    """

    def __init__(self, in_config):
        self.config = in_config
        self.eval_dir = IOUtils.sanitize_path(self.config["EvalDir"]) + "/"
        self.env_dir = IOUtils.sanitize_path(self.config["EnvironmentDataDir"]) + "/"

        if not os.path.exists(self.env_dir):
            os.makedirs(self.env_dir)

        self.num_controllers = len(self.config["Controllers"])
        self.controllers_configs = self.config["Controllers"]

        self.env_config_file = IOUtils.sanitize_path(self.config["EnvironmentConfig"])
        self.env_config = IOUtils.load_toml(self.env_config_file)
        self.cc_params = cc.Parameters(self.env_config_file)

        self.num_steps = self.config["NumSteps"]

        self.plot_map = self.config["PlotMap"]
        self.generate_video = self.config["GenerateVideo"]

        self.feature_file = None
        self.pos_file = None

        file_exists = False

        if "FeatureFile" in self.config and "RobotPosFile" in self.config:
            self.feature_file = self.env_dir + self.config["FeatureFile"]
            self.pos_file = self.env_dir + self.config["RobotPosFile"]
            print(self.feature_file)
            print(self.pos_file)

            if os.path.isfile(self.feature_file):
                feature_file_exists = True
            else:
                print(f"Feature file {self.feature_file} not found")
                feature_file_exists = False

            if os.path.isfile(self.pos_file):
                pos_file_exists = True
            else:
                print(f"Position file {self.pos_file} not found")
                pos_file_exists = False

            file_exists = feature_file_exists and pos_file_exists

        if file_exists is True:
            self.world_idf = WorldIDF(self.cc_params, self.feature_file)
            self.env_main = CoverageSystem(
                self.cc_params, self.world_idf, self.pos_file
            )
        else:
            self.env_main = CoverageSystem(self.cc_params)
            self.world_idf = self.env_main.GetWorldIDF()

            if self.feature_file is not None and self.pos_file is not None:
                self.env_main.WriteEnvironment(self.pos_file, self.feature_file)

    def evaluate(self, save=True):
        cost_data = np.zeros((self.num_controllers, self.num_steps))
        robot_init_pos = self.env_main.GetRobotPositions(force_no_noise=True)

        if self.plot_map:
            map_dir = self.eval_dir + "/plots/"
            os.makedirs(map_dir, exist_ok=True)
            self.env_main.PlotInitMap(map_dir, "InitMap")

        for controller_id in range(self.num_controllers):
            step_count = 0
            env = CoverageSystem(self.cc_params, self.world_idf, robot_init_pos)
            controller_name = self.controllers_configs[controller_id]["Name"]

            if self.generate_video:
                env.RecordPlotData()

            if self.controllers_configs[controller_id]["Type"] == "Learning":
                Controller = ControllerNN
            else:
                Controller = ControllerCVT
            controller = Controller(
                self.controllers_configs[controller_id], self.cc_params, env
            )
            initial_objective_value = env.GetObjectiveValue()
            cost_data[controller_id, step_count] = (
                env.GetObjectiveValue() / initial_objective_value
            )
            step_count = step_count + 1

            while step_count < self.num_steps:
                objective_value, converged = controller.step(env)
                cost_data[controller_id, step_count] = (
                    objective_value / initial_objective_value
                )

                if converged and not self.generate_video:
                    cost_data[controller_id, step_count:] = (
                        objective_value / initial_objective_value
                    )

                    break

                if self.generate_video:
                    env.RecordPlotData()

                step_count = step_count + 1

                if step_count % 100 == 0:
                    print(
                        f"{controller.name} "
                        f"Step {step_count} "
                        f"Obj Value {cost_data[controller_id, step_count - 1]:.3e}"
                    )
            print(
                f"{controller.name} "
                f"final step {step_count} "
                f"Obj Value {cost_data[controller_id, step_count - 1]:.3e}"
            )

            if save is True:
                controller_dir = self.eval_dir + "/" + controller_name

                if not os.path.exists(controller_dir):
                    os.makedirs(controller_dir)

                controller_data_file = controller_dir + "/" + "eval.csv"
                np.savetxt(
                    controller_data_file, cost_data[controller_id, :], delimiter=","
                )

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
    evaluator.evaluate()
