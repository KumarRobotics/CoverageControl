import copy
import torch
from torch.nn import functional as F
from torch_geometric.nn import MLP

from .cnn_backbone import CNNBackBone

class CNN(torch.nn.Module):
    """
    Implements an architecture consisting of a multi-layer CNN followed by an MLP, according to parameters specified in the input config
    This is the current architecture used in the hybrid CNN-GNN
    """
    def __init__(self, config):
        super(CNN, self).__init__()
        self.config = config
        self.parse_config()

        self.cnn_backbone = CNNBackBone(self.config)
        self.linear = torch.nn.Linear(2 * self.output_dim, self.output_dim)
    
    def parse_config(self):
        self.input_dim = self.config["InputDim"]
        self.output_dim = self.config["OutputDim"]
        self.num_layers = self.config["NumLayers"]
        self.latent_size = self.config["LatentSize"]
        self.kernel_size = self.config["KernelSize"]
        self.image_size = self.config["ImageSize"]
    
    def forward(self, x, return_embed=None):
        x = self.cnn_backbone(x)
        x = self.linear(x)
        return x

    def LoadModelCPP(self, model_path):
        jit_model = torch.jit.load(model_path)
        self.load_state_dict(jit_model.state_dict())
