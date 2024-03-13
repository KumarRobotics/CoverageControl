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

class CNNConfigParser():
    def __init__(self):
        pass

    def Parse(self, config: dict) -> None:
        self.config = config
        self.input_dim = self.config['InputDim']
        self.output_dim = self.config['OutputDim']
        self.num_layers = self.config['NumLayers']
        self.latent_size = self.config['LatentSize']
        self.kernel_size = self.config['KernelSize']
        self.image_size = self.config['ImageSize']

class GNNConfigParser():
    def __init__(self, config):
        pass

    def parse(self, config: dict) -> None:
        self.config = config
        self.input_dim = self.config['InputDim']
        self.output_dim = self.config['OutputDim']
        self.num_hops = self.config['NumHops']
        self.num_layers = self.config['NumLayers']
        self.latent_size = self.config['LatentSize']
