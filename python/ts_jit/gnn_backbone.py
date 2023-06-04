import sys
import torch
import math
from torch import nn
from torch_geometric.nn import TAGConv
import yaml

class GNNBackBone(nn.Module):
    """
    Implements a multi-layer graph convolutional neural network, with ReLU non-linearities between layers,
    according to hyperparameters specified in the input config
    """
    def __init__(self, input_dim, num_layers, num_hops, latent_size):
        super().__init__()

        self.input_dim_ = input_dim
        self.num_layers_ = num_layers
        self.num_hops_ = num_hops
        self.latent_size_ = latent_size

        f = [self.latent_size_]*self.num_layers_
        f = [self.input_dim_] + f

        self.graph_convs = nn.ModuleList()
        for layer in range(self.num_layers_):
            self.graph_convs.append(TAGConv(in_channels=f[layer], out_channels=f[layer+1], K=self.num_hops_).jittable())
    
    def forward(self, x, edge_weight) -> torch.Tensor:
        edge_index = edge_weight.indices()
        weight = edge_weight.values()
        for conv in self.graph_convs:
            x = conv(x, edge_index, weight)
            x = torch.relu(x)
        return x

if __name__ == "__main__":
    # Load config yaml file
    config_file = str(sys.argv[1])
    with open(config_file, 'r') as stream:
        config = yaml.safe_load(stream)['GNNBackBone']
    print(config)
    scripted_model = torch.jit.script(GNNBackBone(config['InputDim'], config['NumLayers'], config['NumHops'], config['LatentSize']))
    script_file = config["ScriptName"]
    scripted_model.save(script_file)
