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
Implements an architecture consisting of a multi-layer CNN followed by an MLP, according to parameters specified in the input config
"""
import torch
from torch_geometric.nn import MLP

from .cnn_backbone import CNNBackBone
from .config_parser import CNNConfigParser

__all__ = ["CNN"]


## @ingroup python_api
class CNN(torch.nn.Module, CNNConfigParser):
    """
    Implements an architecture consisting of a multi-layer CNN followed by an MLP, according to parameters specified in the input config.
    """

    def __init__(self, config: dict):
        super().__init__()
        self.parse(config)

        self.cnn_backbone = CNNBackBone(self.config)
        self.mlp = MLP(
            [
                self.latent_size,
                2 * self.latent_size,
                2 * self.latent_size,
                self.latent_size,
            ]
        )
        self.linear = torch.nn.Linear(self.latent_size, self.output_dim)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        Forward pass through the network

        Args:
            x: Input tensor

        Returns:
            Output tensor
        """
        x = self.cnn_backbone(x)
        x = self.mlp(x)
        x = self.linear(x)

        return x

    def load_cpp_model(self, model_path: str) -> None:
        """
        Loads a model saved in cpp jit format
        """
        jit_model = torch.jit.load(model_path)
        self.load_state_dict(jit_model.state_dict(), strict=False)

    def load_model(self, model_path: str) -> None:
        """
        Loads a model saved in pytorch format
        """
        self.load_state_dict(torch.load(model_path), strict=False)
