# @file simple_data_generation.py
# This file contains the code to generate a dataset for learning
#
# Uses the following configuration given in a toml file:
# DataDir =  "${CoverageControl_ws}/datasets/lpac" # Absolute location
# EnvironmentConfig = "${CoverageControl_ws}/datasets/lpac/coverage_control_params.toml" # Absolute location
#
# NumDataset = 1000
#
# # Number of steps to take before data is stores
# # This helps in creating a more diverse dataset
# EveryNumSteps = 5
#
# # The robots stop moving once the algorithm has converged
# # Having some of these converged steps can help in stabilizing robot actions
# ConvergedDataRatio = 0.02
#
# # Resizing of maps and Sparsification of tensors are triggered every TriggerPostProcessing dataset
# # This should be set based on RAM resources available on the system
# TriggerPostProcessing = 100
#
# CNNMapSize = 32
# @file simple_data_generation.py
#  @brief Generates a dataset for coverage control learning
import math
import os
import pathlib
import sys

import coverage_control
import torch
from coverage_control import CoverageSystem
from coverage_control import IOUtils
from coverage_control import CoverageEnvUtils
from coverage_control.algorithms import ClairvoyantCVT as CoverageAlgorithm

# @ingroup python_api


class SimpleDatasetGenerator:
    """
    Class for generating dataset for learning
    """

    def __init__(self, config_file: str, append_dir: str = None):
        """
        Constructor for the class

        Args:
            config_file (str): Configuration file in toml format
            append_dir (str): Name of the directory to append to the dataset directory
        """

        self.config = IOUtils.load_toml(config_file)
        self.data_dir = IOUtils.sanitize_path(self.config["DataDir"])
        self.dataset_dir = self.data_dir + "/data/"

        if append_dir is not None:
            self.dataset_dir += append_dir

        if not pathlib.Path(self.data_dir).exists():
            print(f"{self.data_dir} does not exist")
            exit()

        if not pathlib.Path(self.dataset_dir).exists():
            os.makedirs(self.dataset_dir)

        env_config_file = IOUtils.sanitize_path(self.config["EnvironmentConfig"])
        env_config_file = pathlib.Path(env_config_file)

        if not env_config_file.exists():
            print(f"{env_config_file} does not exist")
            exit()

        self.env_params = coverage_control.Parameters(env_config_file.as_posix())

        # Initialize variables
        self.dataset_count = 0
        self.env_count = 0
        self.trigger_count = 0
        self.trigger_start_idx = 0

        self.num_dataset = self.config["NumDataset"]
        self.num_robots = self.env_params.pNumRobots
        self.comm_range = self.env_params.pCommunicationRange
        self.resolution = self.env_params.pResolution
        self.cnn_map_size = self.config["CNNMapSize"]
        self.every_num_step = self.config["EveryNumSteps"]
        self.trigger_size = self.config["TriggerPostProcessing"]
        self.converged_data_ratio = self.config["ConvergedDataRatio"]

        if self.trigger_size == 0 or self.trigger_size > self.num_dataset:
            self.trigger_size = self.num_dataset

        if torch.cuda.is_available():
            self.device = torch.device("cuda")
        else:
            self.device = torch.device("cpu")

        # Initialize tensors
        self.actions = torch.zeros((self.num_dataset, self.num_robots, 2))
        self.robot_positions = torch.zeros((self.num_dataset, self.num_robots, 2))
        self.raw_local_maps = torch.zeros(
                (
                    self.trigger_size,
                    self.num_robots,
                    self.env_params.pLocalMapSize,
                    self.env_params.pLocalMapSize,
                    )
                )
        self.raw_obstacle_maps = torch.zeros(
                (
                    self.trigger_size,
                    self.num_robots,
                    self.env_params.pLocalMapSize,
                    self.env_params.pLocalMapSize,
                    )
                )
        self.local_maps = torch.zeros(
                (self.num_dataset, self.num_robots, self.cnn_map_size, self.cnn_map_size)
                )
        self.obstacle_maps = torch.zeros(
                (self.num_dataset, self.num_robots, self.cnn_map_size, self.cnn_map_size)
                )
        self.comm_maps = torch.zeros(
                (self.num_dataset, self.num_robots, 2, self.cnn_map_size, self.cnn_map_size)
                )
        self.coverage_features = torch.zeros((self.num_dataset, self.num_robots, 7))
        self.edge_weights = torch.zeros(
                (self.num_dataset, self.num_robots, self.num_robots)
                )

        self.run_data_generation()
        self.trigger_post_processing()
        del self.raw_local_maps
        del self.raw_obstacle_maps
        self.save_dataset()

    def run_data_generation(self):
        num_non_converged_env = 0

        while self.dataset_count < self.num_dataset:
            self.env = CoverageSystem(self.env_params)
            self.alg = CoverageAlgorithm(self.env_params, self.num_robots, self.env)
            self.env_count += 1
            print("Environment: " + str(self.env_count))
            num_steps = 0
            is_converged = False

            while (
                    num_steps < self.env_params.pEpisodeSteps
                    and not is_converged
                    and self.dataset_count < self.num_dataset
                    ):

                if num_steps % self.every_num_step == 0:
                    is_converged = self.step_with_save()
                else:
                    is_converged = self.step_without_save()
                num_steps += 1

            if num_steps == self.env_params.pEpisodeSteps:
                num_non_converged_env += 1
                print("Non-converged environment: " + str(num_non_converged_env))

            print("Converged in " + str(num_steps) + " steps")

            num_converged_data = math.ceil(
                    self.converged_data_ratio * num_steps / self.every_num_step
                    )
            converged_data_count = 0

            while (
                    converged_data_count < num_converged_data
                    and self.dataset_count < self.num_dataset
                    ):
                self.step_with_save()
                converged_data_count += 1

    def step_with_save(self):
        self.alg.ComputeActions()
        converged = self.alg.IsConverged()
        actions = self.alg.GetActions()
        count = self.dataset_count
        self.actions[count] = CoverageEnvUtils.to_tensor(actions)
        self.robot_positions[count] = CoverageEnvUtils.get_robot_positions(self.env)
        self.coverage_features[count] = CoverageEnvUtils.get_voronoi_features(self.env)
        self.raw_local_maps[self.trigger_count] = CoverageEnvUtils.get_raw_local_maps(
                self.env, self.env_params
                )
        self.raw_obstacle_maps[self.trigger_count] = (
                CoverageEnvUtils.get_raw_obstacle_maps(self.env, self.env_params)
                )
        self.comm_maps[count] = CoverageEnvUtils.get_communication_maps(
                self.env, self.env_params, self.cnn_map_size
                )
        self.edge_weights[count] = CoverageEnvUtils.get_weights(
                self.env, self.env_params
                )
        self.dataset_count += 1

        if self.dataset_count % 100 == 0:
            print(f"Dataset: {self.dataset_count}/{self.num_dataset}")

        self.trigger_count += 1

        if self.trigger_count == self.trigger_size:
            self.trigger_post_processing()
            self.trigger_count = 0

        if self.env.StepActions(actions):
            return True

        return converged

    def trigger_post_processing(self):
        if self.trigger_start_idx > self.num_dataset - 1:
            return
        trigger_end_idx = min(
                self.num_dataset, self.trigger_start_idx + self.trigger_size
                )
        raw_local_maps = self.raw_local_maps[
                0 : trigger_end_idx - self.trigger_start_idx
                ]
        raw_local_maps = raw_local_maps.to(self.device)
        resized_local_maps = CoverageEnvUtils.resize_maps(
                raw_local_maps, self.cnn_map_size
                )
        self.local_maps[self.trigger_start_idx : trigger_end_idx] = (
                resized_local_maps.view(
                    -1, self.num_robots, self.cnn_map_size, self.cnn_map_size
                    )
                .cpu()
                .clone()
                )

        raw_obstacle_maps = self.raw_obstacle_maps[
                0 : trigger_end_idx - self.trigger_start_idx
                ]
        raw_obstacle_maps = raw_obstacle_maps.to(self.device)
        resized_obstacle_maps = CoverageEnvUtils.resize_maps(
                raw_obstacle_maps, self.cnn_map_size
                )
        self.obstacle_maps[self.trigger_start_idx : trigger_end_idx] = (
                resized_obstacle_maps.view(
                    -1, self.num_robots, self.cnn_map_size, self.cnn_map_size
                    )
                .cpu()
                .clone()
                )

        self.trigger_start_idx = trigger_end_idx

    def normalize_tensor(self, tensor):
        tensor_mean = tensor.mean(dim=[0, 1])
        tensor_std = tensor.std(dim=[0, 1])
        tensor = (tensor - tensor_mean) / tensor_std

        return tensor, tensor_mean, tensor_std

    def normalize_communication_maps(self):
        min_val = self.comm_maps.min()
        max_val = self.comm_maps.max()
        range_val = max_val - min_val
        self.comm_maps = (self.comm_maps - min_val) / range_val
        print("Communication map min: " + str(min_val))
        print("Communication map max: " + str(max_val))

        return min_val, range_val

    def save_dataset(self):
        dataset_dir_path = pathlib.Path(self.dataset_dir)
        torch.save(self.robot_positions, dataset_dir_path / "robot_positions.pt")
        torch.save(self.local_maps.to_sparse(), dataset_dir_path / "local_maps.pt")
        torch.save(
                self.obstacle_maps.to_sparse(), dataset_dir_path / "obstacle_maps.pt"
                )
        torch.save(self.edge_weights.to_sparse(), dataset_dir_path / "edge_weights.pt")

        torch.save(self.comm_maps.to_sparse(), dataset_dir_path / "comm_maps.pt")

        torch.save(self.actions, dataset_dir_path / "actions.pt")
        torch.save(self.coverage_features, dataset_dir_path / "coverage_features.pt")

    def step_without_save(self):
        self.alg.ComputeActions()
        converged = self.alg.IsConverged()

        if self.env.StepActions(self.alg.GetActions()):
            return True

        return converged

    def get_tensor_byte_size_MB(self, tensor):
        return (tensor.element_size() * tensor.nelement()) / (1024 * 1024)

    def print_tensor_sizes(self, file=sys.stdout):
        # Set to two decimal places
        print("Tensor sizes:", file=file)
        print("Actions:", self.get_tensor_byte_size_MB(self.actions), file=file)
        print(
                "Robot positions:",
                self.get_tensor_byte_size_MB(self.robot_positions),
                file=file,
                )
        print(
                "Raw local maps:",
                self.get_tensor_byte_size_MB(self.raw_local_maps),
                file=file,
                )
        print(
                "Raw obstacle maps:",
                self.get_tensor_byte_size_MB(self.raw_obstacle_maps),
                file=file,
                )
        print("Local maps:", self.get_tensor_byte_size_MB(self.local_maps), file=file)
        print(
                "Obstacle maps:",
                self.get_tensor_byte_size_MB(self.obstacle_maps),
                file=file,
                )
        print("Comm maps:", self.get_tensor_byte_size_MB(self.comm_maps), file=file)
        print(
                "Coverage features:",
                self.get_tensor_byte_size_MB(self.coverage_features),
                file=file,
                )


if __name__ == "__main__":
    config_file = sys.argv[1]

    if len(sys.argv) > 2:
        append_folder = sys.argv[2]
    else:
        append_folder = None
    SimpleDatasetGenerator(config_file, append_folder)
