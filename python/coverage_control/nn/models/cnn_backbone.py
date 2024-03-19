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
Implements a multi-layer convolutional neural network
"""
import torch

from .config_parser import CNNConfigParser

## @ingroup python_api
class CNNBackBone(torch.nn.Module, CNNConfigParser):
    """
    Implements a multi-layer convolutional neural network,
    with leaky-ReLU non-linearities between layers,
    according to hyperparameters specified in the config
    """

    def __init__(self, config: dict):
        super().__init__()
        self.parse(config)

        self.add_module(
            "conv0",
            torch.nn.Conv2d(
                self.input_dim, self.latent_size, kernel_size=self.kernel_size
            ),
        )
        self.add_module("batch_norm0", torch.nn.BatchNorm2d(self.latent_size))

        for layer in range(self.num_layers - 1):
            self.add_module(
                f"conv{layer + 1}",
                torch.nn.Conv2d(
                    self.latent_size, self.latent_size, kernel_size=self.kernel_size
                ),
            )
            self.add_module(
                f"batch_norm{layer + 1}", torch.nn.BatchNorm2d(self.latent_size)
            )

        self.flatten_size = (
            self.latent_size
            * (self.image_size - self.num_layers * (self.kernel_size - 1)) ** 2
        )

        self.add_module(
            "linear_1", torch.nn.Linear(self.flatten_size, self.latent_size)
        )
        # self.add_module("linear_2", torch.nn.Linear(self.latent_size, self.backbone_output_dim))
        # self.add_module("linear_3", torch.nn.Linear(2 * self.output_dim, self.output_dim))

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        Forward pass through the network

        Args:
            x: input tensor
        """
        for layer in range(self.num_layers):
            x = torch.nn.functional.leaky_relu(
                self._modules[f"batch_norm{layer}"](self._modules[f"conv{layer}"](x))
            )
            # x = self._modules["conv{}".format(layer)](x)
            # x = self._modules["batch_norm{}".format(layer)](x)
            # x = torch.nn.functional.leaky_relu(x)

        x = x.flatten(1)
        x = torch.nn.functional.leaky_relu(self.linear_1(x))

        return x
        # x = torch.nn.functional.leaky_relu(self.linear_2(x))
        # x = self.linear_3(x)
        # output = x.reshape(x.shape[0], self.latent_size, -1)
        # output, _ = torch.max(output, dim=2)
        # return output
