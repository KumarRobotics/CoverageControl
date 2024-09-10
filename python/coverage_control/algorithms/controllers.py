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
# @file controller.py
#  @brief Base classes for CVT and neural network based controllers
import coverage_control.nn as cc_nn
import torch
torch.set_float32_matmul_precision('high')

from . import CentralizedCVT
from . import ClairvoyantCVT
from . import DecentralizedCVT
from . import NearOptimalCVT
from .. import IOUtils
from .. import CoverageEnvUtils
from ..core import CoverageSystem
from ..core import Parameters
from ..core import PointVector

__all__ = ["ControllerCVT", "ControllerNN"]


class ControllerCVT:
    """
    Controller class for CVT based controllers
    """

    def __init__(self, config: dict, params: Parameters, env: CoverageSystem):
        """
        Constructor for the CVT controller
        Args:
            config: Configuration dictionary
            params: Parameters object
            env: CoverageSystem object
        """
        self.name = config["Name"]
        self.params = params
        match config["Algorithm"]:
            case "DecentralizedCVT":
                self.alg = DecentralizedCVT(params, env)
            case "ClairvoyantCVT":
                self.alg = ClairvoyantCVT(params, env)
            case "CentralizedCVT":
                self.alg = CentralizedCVT(params, env)
            case "NearOptimalCVT":
                self.alg = NearOptimalCVT(params, env)
            case _:
                raise ValueError(f"Unknown controller type: {controller_type}")

    def step(self, env: CoverageSystem) -> (float, bool):
        """
        Step function for the CVT controller

        Performs three steps:
        1. Compute actions using the CVT algorithm
        2. Get the actions from the algorithm
        3. Step the environment using the actions
        Args:
            env: CoverageSystem object
        Returns:
            Objective value and convergence flag
        """
        self.alg.ComputeActions()
        actions = self.alg.GetActions()
        converged = self.alg.IsConverged()
        error_flag = env.StepActions(actions)

        if error_flag:
            raise ValueError("Error in step")

        return env.GetObjectiveValue(), converged


class ControllerNN:
    """
    Controller class for neural network based controllers
    """

    def __init__(self, config: dict, params: Parameters, env: CoverageSystem):
        """
        Constructor for the neural network controller
        Args:
            config: Configuration dictionary
            params: Parameters object
            env: CoverageSystem object
        """
        self.config = config
        self.params = params
        self.name = self.config["Name"]
        self.use_cnn = self.config["UseCNN"]
        self.use_comm_map = self.config["UseCommMap"]
        self.cnn_map_size = self.config["CNNMapSize"]

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        # print(f"Using device: {self.device}")

        if "ModelFile" in self.config:
            self.model_file = IOUtils.sanitize_path(self.config["ModelFile"])
            self.model = torch.load(self.model_file).to(self.device)
        else:  # Load from ModelStateDict
            self.learning_params_file = IOUtils.sanitize_path(
                    self.config["LearningParams"]
                    )
            self.learning_params = IOUtils.load_toml(self.learning_params_file)
            self.model = cc_nn.LPAC(self.learning_params).to(self.device)
            self.model.load_model(IOUtils.sanitize_path(self.config["ModelStateDict"]))

        self.actions_mean = self.model.actions_mean.to(self.device)
        self.actions_std = self.model.actions_std.to(self.device)
        self.model = self.model.to(self.device)
        self.model.eval()
        self.model = torch.compile(self.model, dynamic=True)

    def step(self, env):
        """
        step function for the neural network controller

        Performs three steps:
        1. Get the data from the environment
        2. Get the actions from the model
        3. Step the environment using the actions
        Args:
            env: CoverageSystem object
        Returns:
            Objective value and convergence flag
        """
        pyg_data = CoverageEnvUtils.get_torch_geometric_data(
                env, self.params, True, self.use_comm_map, self.cnn_map_size
                ).to(self.device)
        with torch.no_grad():
            actions = self.model(pyg_data)
        actions = actions * self.actions_std + self.actions_mean
        point_vector_actions = PointVector(actions.cpu().numpy())
        env.StepActions(point_vector_actions)

        # Check if actions are all zeros (1e-12)
        if torch.allclose(actions, torch.zeros_like(actions), atol=1e-5):
            return env.GetObjectiveValue(), True
        return env.GetObjectiveValue(), False
