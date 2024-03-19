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
Implements a GNN architecture.
"""
import torch
import torch_geometric

from .config_parser import GNNConfigParser

## @ingroup python_api
class GNNBackBone(torch.nn.Module, GNNConfigParser):
    """
    Implements a GNN architecture,
    according to hyperparameters specified in the input config
    """

    def __init__(self, config, input_dim=None):
        super().__init__()

        self.parse(config)

        if input_dim is not None:
            self.input_dim = input_dim

        self.add_module(
            "graph_conv_0",
            torch_geometric.nn.TAGConv(
                in_channels=self.input_dim,
                out_channels=self.latent_size,
                K=self.num_hops,
            ),
        )

        for i in range(1, self.num_layers):
            self.add_module(
                "graph_conv_{}".format(i),
                torch_geometric.nn.TAGConv(
                    in_channels=self.latent_size,
                    out_channels=self.latent_size,
                    K=self.num_hops,
                ),
            )

    def forward(
        self, x: torch.Tensor, edge_index: torch.Tensor, edge_weight=None
    ) -> torch.Tensor:
        for i in range(self.num_layers):
            x = self._modules["graph_conv_{}".format(i)](x, edge_index, edge_weight)
            x = torch.relu(x)

        return x
