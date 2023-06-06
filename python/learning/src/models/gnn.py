import torch
from torch_geometric.nn import MLP

from .config_parser import GNNConfigParser
from .gnn_backbone import GNNBackBone
from .cnn_backbone import CNNBackBone

class CNNGNN(torch.nn.Module, GNNConfigParser):
    def __init__(self, config):
        super(CNNGNN, self).__init__()
        self.cnn_config = config['CNN']
        self.Parse(config['GNN'])
        self.cnn_backbone = CNNBackBone(self.cnn_config)
        self.gnn_backbone = GNNBackBone(self.config, self.cnn_backbone.backbone_output_dim)
        self.mlp = MLP([self.latent_size, 32, 32], norm=None)
        self.output_linear = torch.nn.Linear(32, self.output_dim)

    def forward(self, data):
        x, edge_index, edge_weight = data.x, data.edge_index, data.edge_weight
        x = self.cnn_backbone(x)
        x = self.gnn_backbone(x, edge_index, edge_weight)
        x = self.mlp(x)
        x = self.output_linear(x)
        return x

    def LoadCNNBackBone(self, model_path):
        self.load_state_dict(torch.load(model_path), strict=False)
