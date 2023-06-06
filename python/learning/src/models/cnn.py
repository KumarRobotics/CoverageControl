import torch
from .cnn_backbone import CNNBackBone
from .config_parser import CNNConfigParser

class CNN(torch.nn.Module, CNNConfigParser):
    """
    Implements an architecture consisting of a multi-layer CNN followed by an MLP, according to parameters specified in the input config
    This is the current architecture used in the hybrid CNN-GNN
    """
    def __init__(self, config):
        super(CNN, self).__init__()
        self.Parse(config)

        self.cnn_backbone = CNNBackBone(self.config)
        self.linear = torch.nn.Linear(self.backbone_output_dim, self.output_dim)
    
    def forward(self, x, return_embed=None):
        x = self.cnn_backbone(x)
        x = self.linear(x)
        return x

    def LoadModelCPP(self, model_path):
        jit_model = torch.jit.load(model_path)
        self.load_state_dict(jit_model.state_dict(), strict=False)

    def LoadModel(self, model_path):
        self.load_state_dict(torch.load(model_path), strict=False)
