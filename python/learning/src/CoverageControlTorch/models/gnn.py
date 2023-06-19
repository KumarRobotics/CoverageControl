import torch
from torch_geometric.nn import MLP

from CoverageControlTorch.models.config_parser import GNNConfigParser
from CoverageControlTorch.models.gnn_backbone import GNNBackBone
from CoverageControlTorch.models.cnn_backbone import CNNBackBone

class CNNGNN(torch.nn.Module, GNNConfigParser):
    def __init__(self, config):
        super(CNNGNN, self).__init__()
        self.cnn_config = config['CNN']
        self.Parse(config['GNN'])
        self.cnn_backbone = CNNBackBone(self.cnn_config)
        self.gnn_backbone = GNNBackBone(self.config, self.cnn_backbone.latent_size)
        # self.mlp = MLP([self.latent_size, 32, self.output_dim], norm=None)
        self.gnn_mlp = MLP([self.latent_size, 32, 32])
        self.output_linear = torch.nn.Linear(32, self.output_dim)

    def forward(self, data):
        x, edge_index, edge_weight = data.x, data.edge_index, data.edge_weight
        cnn_output = self.cnn_backbone(x.view(-1, x.shape[-3], x.shape[-2], x.shape[-1]))
        gnn_output = self.gnn_backbone(cnn_output, edge_index, edge_weight)
        mlp_output = self.gnn_mlp(gnn_output)
        x = self.output_linear(mlp_output)
        return x

    def LoadCNNBackBone(self, model_path):
        self.load_state_dict(torch.load(model_path).state_dict(), strict=False)
