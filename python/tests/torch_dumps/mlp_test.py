import torch
from mlp import MLP1
import CoverageControlTorch.utils.coverage_system as CoverageSystemUtils
from CoverageControlTorch.data_loaders import data_loader_utils as dl_utils

if torch.cuda.is_available():
    device = torch.device('cpu')
    print('Using GPU')

# nlayers = 2
# K = 2
# num_nodes = 3
# nfeatures = 2
# latent_size = 3
class GNN(torch.nn.Module):
    def __init__(self):
        super(GNN, self).__init__()
        self.add_module("graph_conv_0", TAGConv1(in_channels = nfeatures, out_channels = latent_size, K = 2))
        self.add_module("graph_conv_1", TAGConv1(in_channels = latent_size, out_channels = latent_size, K = 2))

    def forward(self, x, edge_index, edge_weight = None):
        x = self._modules["graph_conv_0"](x, edge_index, edge_weight)
        x = torch.relu(x)
        x = self._modules["graph_conv_1"](x, edge_index, edge_weight)
        x = torch.relu(x)
        return x

# model = GNN()
nlayers = 5
K = 3
num_nodes = 10
nfeatures = 34
latent_size = 256

model = torch.load('model_k3_1024.pt')
model.eval()
model.to(device)

print(model)
gnn = model
gnn = model.gnn_backbone
print(gnn)

mlp_layer_custom = MLP1([256,32,32])
mlp_layer_orig = model.gnn_mlp
mlp_layer_custom.load_state_dict(mlp_layer_orig.state_dict())
mlp_layer = mlp_layer_custom
print(mlp_layer.state_dict().keys())
mlp_layer.eval()
# Turn grad off
for param in mlp_layer.parameters():
    param.requires_grad = False
mlp_layer.to('cpu')

# print(mlp_layer.norms[0].module.weight)
random_input = torch.rand((1,256))
# print(random_input)

output1 = mlp_layer(random_input)
# print(f'output1: {output1}')
# print(mlp_layer.norms[0].module.weight)

x = random_input
l0_wts = mlp_layer.lins[0].weight
l0_bias = mlp_layer.lins[0].bias
x = torch.matmul(x, l0_wts.t()) + l0_bias
# print(f'x after l0: {x}')
batch_norm = torch.nn.BatchNorm1d(32)
batch_state_dict = mlp_layer.norms[0].module.state_dict()
print(batch_state_dict)
batch_state_dict['num_batches_tracked'] = batch_state_dict['num_batches_tracked']*0
# batch_state_dict['running_mean'] = batch_state_dict['running_mean']*0
# batch_state_dict['running_var'] = batch_state_dict['running_var']*0

batch_norm.load_state_dict(batch_state_dict)
print(batch_norm.state_dict())
batch_norm.eval()

# n0_wts = mlp_layer.norms[0].module.weight
# n0_bias = mlp_layer.norms[0].module.bias
# x = x*n0_wts + n0_bias
x = batch_norm(x)
# print(f'norm layer: ', {mlp_layer.norms[0].module})
# print(f'x after n0: {x}')
x = torch.relu(x)

l1_wts = mlp_layer.lins[1].weight
l1_bias = mlp_layer.lins[1].bias
x = torch.matmul(x, l1_wts.t()) + l1_bias

output2 = x
# print(f'output2: {output2}')
print(torch.allclose(output1, output2))
# print(x)



output3 = model.output_linear(mlp_layer_orig(random_input))
outlayer_wts = model.output_linear.weight
outlayer_bias = model.output_linear.bias
output4 = torch.matmul(output2, outlayer_wts.t()) + outlayer_bias

print(torch.allclose(output3, output4))
