# @file eval.py
#  @brief Evaluates the performance of the controllers on a set of environments
import os
import argparse

import coverage_control as cc
import numpy as np
from rich.progress import (
    Progress,
    BarColumn,
    TextColumn,
    TimeRemainingColumn,
    TimeElapsedColumn,
    TaskProgressColumn,
    MofNCompleteColumn,
)
from coverage_control import CoverageSystem
from coverage_control import IOUtils
from coverage_control import WorldIDF
from coverage_control.algorithms import ControllerCVT
from coverage_control.algorithms import ControllerNN


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
        self.num_features = self.cc_params.pNumGaussianFeatures
        self.num_envs = self.config["NumEnvironments"]
        self.num_steps = self.config["NumSteps"]
        self.every_num_steps = self.config["EveryNumSteps"]

        self.columns = [
            BarColumn(bar_width=None),
            TaskProgressColumn(),
            TextColumn("[progress.description]{task.description}"),
            MofNCompleteColumn(),
            TextColumn("{task.fields[info]}"),
            TextColumn("Step: {task.fields[step_count]:>4}"),
            TextColumn("Obj: {task.fields[obj]:.2e}"),
            TimeRemainingColumn(),
            TimeElapsedColumn(),
        ]

    def get_normalized_objective_value(self, env, initial_objective_value):
        """
        Returns the normalized objective value of the environment
        :param env: The environment object
        :param initial_objective_value: The initial objective value of the environment
        :return: The normalized objective value
        """
        objective_value = env.GetObjectiveValue()
        return objective_value / initial_objective_value

    def evaluate(self, save=True):
        total_samples = self.num_steps // self.every_num_steps
        if self.every_num_steps != 1:
            total_samples += 1
        cost_data = np.zeros((self.num_controllers, self.num_envs, total_samples))
        progress_update_rate = self.every_num_steps
        if self.every_num_steps < 10:
            progress_update_rate = self.every_num_steps * (10 // self.every_num_steps)

        with Progress(*self.columns, expand=True) as progress:
            task = progress.add_task(
                "[bold blue]Evaluation",
                total=self.num_envs,
                info="Initializing...",
                step_count=0,
                obj=np.nan,
                auto_refresh=False,
            )

            for env_count in range(self.num_envs):
                pos_file = self.env_dir + "/" + str(env_count) + ".pos"
                env_file = self.env_dir + "/" + str(env_count) + ".env"

                if os.path.isfile(env_file) and os.path.isfile(pos_file):
                    world_idf = WorldIDF(self.cc_params, env_file)
                    env_main = CoverageSystem(self.cc_params, world_idf, pos_file)
                else:
                    # print(f"Creating new environment {env_count}")
                    env_main = CoverageSystem(self.cc_params)
                    env_main.WriteEnvironment(pos_file, env_file)
                    world_idf = env_main.GetWorldIDFObject()

                robot_init_pos = env_main.GetRobotPositions(force_no_noise=True)

                for controller_id in range(self.num_controllers):
                    sample_count = 0
                    converged = False
                    env = CoverageSystem(self.cc_params, world_idf, robot_init_pos)

                    if self.controllers_configs[controller_id]["Type"] == "Learning":
                        Controller = ControllerNN
                    else:
                        Controller = ControllerCVT
                    controller = Controller(
                        self.controllers_configs[controller_id], self.cc_params, env
                    )
                    initial_objective_value = env.GetObjectiveValue()
                    cost_data[controller_id, env_count, sample_count] = self.get_normalized_objective_value(env, initial_objective_value)
                    sample_count += 1

                    for step_count in range(1, self.num_steps):
                        converged = controller.step(env)
                        if (step_count + 1) % self.every_num_steps == 0:
                            normalized_objective_value = self.get_normalized_objective_value(env, initial_objective_value)
                            cost_data[controller_id, env_count, sample_count] = normalized_objective_value
                            sample_count += 1
                            if (step_count + 1) % progress_update_rate == 0:
                                info = f"Controller {controller_id}/{self.num_controllers}: {controller.name} "
                                progress.update(
                                    task,
                                    info=info,
                                    step_count=step_count+1,
                                    obj=normalized_objective_value,
                                )
                                progress.refresh()

                        if converged:
                            cost_data[controller_id, env_count, sample_count:] = self.get_normalized_objective_value(env, initial_objective_value)
                            step_count = self.num_steps
                            sample_count = total_samples

                            break

                    if controller_id == self.num_controllers - 1:
                        info = f"Controller {controller_id + 1}/{self.num_controllers}: {controller.name} "
                        progress.update(task, info=info)
                        progress.refresh()

                    if save is True:
                        self.controller_dir = (
                            self.eval_dir
                            + "/"
                            + self.controllers_configs[controller_id]["Name"]
                        )
                        controller_data_file = self.controller_dir + "/" + "eval.csv"
                        np.savetxt(
                            controller_data_file,
                            cost_data[controller_id, : env_count + 1, :],
                            delimiter=",",
                        )
                    # env.RenderRecordedMap(self.eval_dir + "/" + self.controllers[controller_id]["Name"] + "/", "video.mp4")
                    del controller
                    del env
                progress.advance(task)
                progress.refresh()

        return cost_data


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("config_file", type=str, help="Path to config file")
    args = parser.parse_args()
    config = IOUtils.load_toml(args.config_file)

    evaluator = Evaluator(config)
    evaluator.evaluate()
