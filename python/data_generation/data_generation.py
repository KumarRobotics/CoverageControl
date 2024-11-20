# @file data_generation.py
# This file contains the code to generate a dataset for learning
#
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
# SaveAsSparseQ = true
# NormalizeQ = true
#
# [DataSetSplit]
# TrainRatio = 0.7
# ValRatio =  0.2
# TestRatio = 0.1
# @file data_generation.py
#  @brief Class to generate CoverageControl dataset for LPAC architecture
import datetime
import math
import os
import pathlib
import sys
import argparse
from distutils.util import strtobool

from rich.progress import (
    Progress,
    BarColumn,
    TextColumn,
    TimeRemainingColumn,
    TimeElapsedColumn,
    TaskProgressColumn,
    MofNCompleteColumn,
)
import coverage_control
import torch
from coverage_control import CoverageSystem
from coverage_control import IOUtils
from coverage_control import CoverageEnvUtils
from coverage_control.algorithms import ClairvoyantCVT
from coverage_control.algorithms import CentralizedCVT

# @ingroup python_api


class DatasetGenerator:
    """
    Class to generate CoverageControl dataset for LPAC architecture.
    """

    def __init__(self, args):
        self.config_file = args.config_file
        self.config = IOUtils.load_toml(self.config_file)
        self.split_dataset = bool(strtobool(args.split))

        self.algorithm = "ClairvoyantCVT"

        if args.algorithm:
            print("Using CentralizedCVT algorithm")
            self.algorithm = args.algorithm

        self.data_dir = pathlib.Path(
            IOUtils.sanitize_path(self.config["DataDir"]))

        if not self.data_dir.exists():
            print(f"{self.data_dir} does not exist")
            sys.exit()

        if args.append_dir:
            self.dataset_dir = self.data_dir / args.append_dir

        self.dataset_dir_path = pathlib.Path(self.dataset_dir)

        if not self.dataset_dir_path.exists():
            os.makedirs(self.dataset_dir)

        env_config_file = IOUtils.sanitize_path(
            self.config["EnvironmentConfig"])
        env_config_file = pathlib.Path(env_config_file)

        if not env_config_file.exists():
            print(f"{env_config_file} does not exist")
            sys.exit()

        self.env_params = coverage_control.Parameters(
            env_config_file.as_posix())

        # Initialize variables
        self.dataset_count = 0
        self.non_converged_dataset_count = 0
        self.converged_dataset_count = 0
        self.env_count = 0
        self.trigger_count = 0
        self.trigger_start_idx = 0

        self.num_dataset = self.config["NumDataset"]
        self.converged_data_ratio = self.config["ConvergedDataRatio"]
        self.num_converged_dataset = math.ceil(
            self.converged_data_ratio * self.num_dataset
        )
        self.num_non_converged_dataset = self.num_dataset - self.num_converged_dataset
        self.num_robots = self.env_params.pNumRobots
        self.comm_range = self.env_params.pCommunicationRange
        self.resolution = self.env_params.pResolution
        self.cnn_map_size = self.config["CNNMapSize"]
        self.every_num_step = self.config["EveryNumSteps"]

        if "SaveObjective" in self.config:
            self.save_objective = self.config["SaveObjective"]
        else:
            self.save_objective = False

        if "TimeStep" in self.config:
            self.env_params.pTimeStep = self.config["TimeStep"]

        self.trigger_size = self.config["TriggerPostProcessing"]

        if self.trigger_size == 0 or self.trigger_size > self.num_dataset:
            self.trigger_size = self.num_dataset

        if torch.cuda.is_available():
            self.device = torch.device("cuda")
        else:
            self.device = torch.device("cpu")

        # Initialize tensors
        self.actions = torch.zeros((self.num_dataset, self.num_robots, 2))
        self.robot_positions = torch.zeros(
            (self.num_dataset, self.num_robots, 2))
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
            (self.num_dataset, self.num_robots,
             self.cnn_map_size, self.cnn_map_size)
        )
        self.obstacle_maps = torch.zeros(
            (self.num_dataset, self.num_robots,
             self.cnn_map_size, self.cnn_map_size)
        )
        self.comm_maps = torch.zeros(
            (self.num_dataset, self.num_robots, 2,
             self.cnn_map_size, self.cnn_map_size)
        )
        self.coverage_features = torch.zeros(
            (self.num_dataset, self.num_robots, 7))
        self.edge_weights = torch.zeros(
            (self.num_dataset, self.num_robots, self.num_robots)
        )

        if self.save_objective:
            self.objectives = torch.zeros(self.num_dataset)

        self.start_time = datetime.datetime.now()
        # Write metrics
        self.metrics_file = self.dataset_dir_path / "metrics.txt"
        # self.metrics = open(self.metrics_file, 'w')
        with open(self.metrics_file, "w", encoding="utf-8") as f:
            # f.write("Time: " + str(datetime.datetime.now()) + "\n")
            f.write(f"Time: {self.start_time}\n")
            f.write("Dataset directory: " + str(self.dataset_dir) + "\n")
            self.print_tensor_sizes(f)
            f.flush()

        self.print_tensor_sizes()

        columns = [
            BarColumn(bar_width=None),
            TaskProgressColumn(),
            TextColumn("[progress.description]{task.description}"),
            MofNCompleteColumn(),
            TextColumn("#Envs: {task.fields[num_envs]}"),
            TimeRemainingColumn(),
            TimeElapsedColumn(),
        ]
        with Progress(*columns, expand=True) as self.progress:
            self.task = self.progress.add_task(
                "[bold blue]Generating dataset",
                total=self.num_dataset,
                num_envs="",
            )

            self.run_data_generation()

        self.save_dataset()
        end_time = datetime.datetime.now()
        with open(self.metrics_file, "a", encoding="utf-8") as f:
            f.write("Time: " + str(datetime.datetime.now()) + "\n")
            f.write("Total time: " + str(end_time - self.start_time) + "\n")

    def run_data_generation(self):
        num_non_converged_env = 0

        while self.dataset_count < self.num_dataset:
            self.env = CoverageSystem(self.env_params)

            if self.algorithm == "CentralizedCVT":
                self.alg = CentralizedCVT(
                    self.env_params, self.num_robots, self.env)
            else:
                self.alg = ClairvoyantCVT(
                    self.env_params, self.num_robots, self.env)

            self.env_count += 1
            self.progress.update(
                self.task,
                num_envs=f"{self.env_count}"
            )
            self.progress.refresh()
            num_steps = 0
            is_converged = False

            while (
                num_steps < self.env_params.pEpisodeSteps
                and not is_converged
                and self.dataset_count < self.num_dataset
            ):

                if num_steps % self.every_num_step == 0 and self.non_converged_dataset_count < self.num_non_converged_dataset:
                    is_converged = self.step_with_save()
                    self.progress.advance(self.task, advance=1)
                    self.non_converged_dataset_count += 1
                else:
                    is_converged = self.step_without_save()
                num_steps += 1

            if num_steps == self.env_params.pEpisodeSteps:
                num_non_converged_env += 1

            if self.converged_dataset_count < self.num_converged_dataset and self.dataset_count < self.num_dataset:
                self.step_with_save()
                self.progress.advance(self.task, advance=1)
                self.converged_dataset_count += 1

    def step_with_save(self):
        self.alg.ComputeActions()
        converged = self.alg.IsConverged()
        actions = self.alg.GetActions()
        count = self.dataset_count
        self.actions[count] = CoverageEnvUtils.to_tensor(actions)
        self.robot_positions[count] = CoverageEnvUtils.get_robot_positions(
            self.env)
        self.coverage_features[count] = CoverageEnvUtils.get_voronoi_features(
            self.env)
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

        if self.save_objective:
            self.objectives[count] = self.env.GetObjectiveValue()
        self.dataset_count += 1

        # if self.dataset_count % 100 == 0:
        #     print(f"Dataset: {self.dataset_count}/{self.num_dataset}")
        #     print(f"Elapsed time: {datetime.datetime.now() - self.start_time}")

        self.trigger_count += 1

        if self.trigger_count == self.trigger_size:
            self.trigger_post_processing()
            self.trigger_count = 0

        error_flag = self.env.StepActions(actions)

        return converged or error_flag

    def trigger_post_processing(self):
        if self.trigger_start_idx > self.num_dataset - 1:
            return
        trigger_end_idx = min(
            self.num_dataset, self.trigger_start_idx + self.trigger_size
        )
        raw_local_maps = self.raw_local_maps[
            0: trigger_end_idx - self.trigger_start_idx
        ]
        raw_local_maps = raw_local_maps.to(self.device)
        resized_local_maps = CoverageEnvUtils.resize_maps(
            raw_local_maps, self.cnn_map_size
        )
        self.local_maps[self.trigger_start_idx: trigger_end_idx] = (
            resized_local_maps.view(
                -1, self.num_robots, self.cnn_map_size, self.cnn_map_size
            )
            .cpu()
            .clone()
        )

        raw_obstacle_maps = self.raw_obstacle_maps[
            0: trigger_end_idx - self.trigger_start_idx
        ]
        raw_obstacle_maps = raw_obstacle_maps.to(self.device)
        resized_obstacle_maps = CoverageEnvUtils.resize_maps(
            raw_obstacle_maps, self.cnn_map_size
        )
        self.obstacle_maps[self.trigger_start_idx: trigger_end_idx] = (
            resized_obstacle_maps.view(
                -1, self.num_robots, self.cnn_map_size, self.cnn_map_size
            )
            .cpu()
            .clone()
        )

        self.trigger_start_idx = trigger_end_idx

    def normalize_tensor(self, tensor, epsilon=1e-6, zero_mean=False, is_symmetric=False):
        dimensions = torch.tensor(range(tensor.dim()))[:-1]

        if zero_mean is True:
            tensor_mean = torch.zeros_like(dimensions)
        else:
            tensor_mean = tensor.mean(dim=list(dimensions))
        tensor_std = tensor.std(dim=list(dimensions))
        # Set tensor_std to be average of stds and keepdim=True to broadcast

        if is_symmetric is True:
            tensor_std = torch.ones_like(tensor_std) * tensor_std.mean()

        if torch.isnan(tensor_std).any():
            print("NaN in tensor std")

        if torch.isnan(tensor_mean).any():
            print("NaN in tensor mean")
        # Check for division by zero and print warnin

        if torch.any(tensor_std < epsilon):
            print("Tensor: ", tensor_std)
            print("normalize_tensor Warning: Division by zero in normalization")
            print("Adding epsilon to std with zero values")
            tensor_std = torch.where(tensor_std < epsilon, epsilon, tensor_std)
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

    def save_tensor(self, tensor, name, as_sparse=False):
        if self.split_dataset:
            self.save_tensor_split(tensor, name, as_sparse)
        else:
            self.save_tensor_nosplit(tensor, name, as_sparse)

    def save_tensor_nosplit(self, tensor, name, as_sparse=False):
        tensor = tensor.cpu()

        if as_sparse:
            tensor = tensor.to_sparse()

        dataset_dir_path = pathlib.Path(self.dataset_dir)
        torch.save(tensor, dataset_dir_path / name)

    def save_tensor_split(self, tensor, name, as_sparse=False):
        tensor = tensor.cpu()
        train_tensor = tensor[0: self.train_size].clone()
        validation_tensor = tensor[
            self.train_size: self.train_size + self.validation_size
        ].clone()
        test_tensor = tensor[self.train_size + self.validation_size:].clone()

        if as_sparse:
            train_tensor = train_tensor.to_sparse()
            validation_tensor = validation_tensor.to_sparse()
            test_tensor = test_tensor.to_sparse()

        dataset_dir_path = pathlib.Path(self.dataset_dir)
        torch.save(train_tensor, dataset_dir_path / "train/" / name)
        torch.save(validation_tensor, dataset_dir_path / "val/" / name)
        torch.save(test_tensor, dataset_dir_path / "test/" / name)

    def save_dataset(self):
        as_sparse = self.config["SaveAsSparseQ"]

        if not os.path.exists(self.dataset_dir):
            os.makedirs(self.dataset_dir)

        if self.split_dataset:
            self.train_size = int(
                self.num_dataset * self.config["DataSetSplit"]["TrainRatio"]
            )
            self.validation_size = int(
                self.num_dataset * self.config["DataSetSplit"]["ValRatio"]
            )
            self.test_size = self.num_dataset - self.train_size - self.validation_size

            # Make sure the folder exists

            if not os.path.exists(self.dataset_dir / "train"):
                os.makedirs(self.dataset_dir / "train")

            if not os.path.exists(self.dataset_dir / "val"):
                os.makedirs(self.dataset_dir / "val")

            if not os.path.exists(self.dataset_dir / "test"):
                os.makedirs(self.dataset_dir / "test")

        self.save_tensor(self.robot_positions, "robot_positions.pt")
        self.save_tensor(self.local_maps, "local_maps.pt", as_sparse)
        self.save_tensor(self.obstacle_maps, "obstacle_maps.pt", as_sparse)
        self.save_tensor(self.edge_weights, "edge_weights.pt", as_sparse)

        # min_val, range_val = self.normalize_communication_maps()
        self.save_tensor(self.comm_maps, "comm_maps.pt", as_sparse)
        # torch.save(min_val, self.dataset_dir / 'comm_maps_min.pt')
        # torch.save(range_val, self.dataset_dir / 'comm_maps_range.pt')

        self.save_tensor(self.actions, "actions.pt")
        self.save_tensor(self.coverage_features, "coverage_features.pt")
        if self.save_objective:
            self.save_tensor(self.objectives, "objectives.pt")

        if self.config["NormalizeQ"]:
            normalized_actions, actions_mean, actions_std = self.normalize_tensor(
                self.actions, is_symmetric=True, zero_mean=True
            )
            self.save_tensor(normalized_actions, "normalized_actions.pt")
            torch.save(actions_mean, self.dataset_dir_path / "actions_mean.pt")
            torch.save(actions_std, self.dataset_dir_path / "actions_std.pt")
            if self.save_objective:
                normalize_objectives, objectives_mean, objectives_std = self.normalize_tensor(
                    self.objectives
                )
                self.save_tensor(normalize_objectives, "normalized_objectives.pt")
                torch.save(
                    objectives_mean, self.dataset_dir_path / "objectives_mean.pt"
                )
                torch.save(
                    objectives_std, self.dataset_dir_path / "objectives_std.pt"
                )

            # coverage_features, coverage_features_mean, coverage_features_std = (
            #     self.normalize_tensor(self.coverage_features)
            # )
            # self.save_tensor(coverage_features, "normalized_coverage_features.pt")
            # torch.save(
            #     coverage_features_mean,
            #     self.dataset_dir_path / "coverage_features_mean.pt",
            # )
            # torch.save(
            #     coverage_features_std,
            #     self.dataset_dir_path / "coverage_features_std.pt",
            # )

    def step_without_save(self):
        self.alg.ComputeActions()
        converged = self.alg.IsConverged()

        if self.env.StepActions(self.alg.GetActions()):
            return True

        return converged

    def get_tensor_byte_size_mb(self, tensor):
        return (tensor.element_size() * tensor.nelement()) / (1024 * 1024)

    def print_tensor_sizes(self, file=sys.stdout):
        # Set to two decimal places
        print("Tensor sizes:", file=file)
        print("Actions:", self.get_tensor_byte_size_mb(self.actions), file=file)
        print(
            "Robot positions:",
            self.get_tensor_byte_size_mb(self.robot_positions),
            file=file,
        )
        print(
            "Raw local maps:",
            self.get_tensor_byte_size_mb(self.raw_local_maps),
            file=file,
        )
        print(
            "Raw obstacle maps:",
            self.get_tensor_byte_size_mb(self.raw_obstacle_maps),
            file=file,
        )
        print("Local maps:", self.get_tensor_byte_size_mb(
            self.local_maps), file=file)
        print(
            "Obstacle maps:",
            self.get_tensor_byte_size_mb(self.obstacle_maps),
            file=file,
        )
        print("Comm maps:", self.get_tensor_byte_size_mb(
            self.comm_maps), file=file)
        print(
            "Coverage features:",
            self.get_tensor_byte_size_mb(self.coverage_features),
            file=file,
        )


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("config_file", type=str, help="Path to config file")
    parser.add_argument("--append-dir",
                        type=str,
                        default="data",
                        help="Append directory to dataset path",
                        required=False
                        )
    parser.add_argument("--split",
                        type=str,
                        default="False",
                        help="Split dataset into train, validation and test sets",
                        required=False
                        )
    parser.add_argument("--algorithm",
                        type=str,
                        help="Algorithm for generating dataset",
                        required=False,
                        )
    args = parser.parse_args()
    DatasetGenerator(args)
