import os
import sys
import yaml
import torch
import torch_geometric
import CoverageControlTorch
from CoverageControlTorch.models.config_parser import GNNConfigParser

class GNNBackBone(torch.nn.Module, GNNConfigParser):
    """
    Implements a multi-layer graph convolutional neural network, with ReLU non-linearities between layers,
    according to hyperparameters specified in the input config
    """
    def __init__(self, config, input_dim = None):
        super(GNNBackBone, self).__init__()

        self.Parse(config)
        if input_dim is not None:
            self.input_dim = input_dim

        self.add_module("graph_conv_0", torch_geometric.nn.TAGConv(in_channels = self.input_dim, out_channels = self.latent_size, K = self.num_hops))
        for i in range(1, self.num_layers):
            self.add_module("graph_conv_{}".format(i), torch_geometric.nn.TAGConv(in_channels = self.latent_size, out_channels = self.latent_size, K = self.num_hops))


    def forward(self, x, edge_index, edge_weight = None):
        for i in range(self.num_layers):
            x = self._modules["graph_conv_{}".format(i)](x, edge_index, edge_weight)
            x = torch.relu(x)
        return x


