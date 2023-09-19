import torch
import CoverageControlTorch.utils.coverage_system as CoverageSystemUtils
from CoverageControlTorch.data_loaders import data_loader_utils as dl_utils
from tagconv1 import TAGConv1

class GNN(torch.nn.Module):
    def __init__(self):
        super(GNN, self).__init__()
        self.add_module("graph_conv_0", TAGConv1(in_channels = 2, out_channels = 1, K = 2))

    def forward(self, x, edge_index, edge_weight = None):
        x = self._modules["graph_conv_0"](x, edge_index, edge_weight)
        x = torch.relu(x)
        return x

device = torch.device('cpu')

gnn = GNN()
print(gnn.graph_conv_0)
print(gnn.graph_conv_0.state_dict().keys())
print(gnn.graph_conv_0.bias)
print(gnn.graph_conv_0.lins[0].weight)
print(gnn.graph_conv_0.lins[1].weight)



nlayers = 1
K = 2
num_nodes = 3
nfeatures = 2
# Create a feature input tensor for num_nodes nodes with nfeafeatures
features = torch.randn(num_nodes, nfeatures)
features = torch.tensor([[1, 2], [3, 4], [5, 6]]).to(torch.float32)
# Create a binary graph adjacency matrix for num_nodes nodes
adj_mat = torch.randint(0, 2, (num_nodes, num_nodes))
# Make the adjacency matrix symmetric
adj_mat = adj_mat + adj_mat.t()
adj_mat[adj_mat > 0] = 1
adj_mat = adj_mat.fill_diagonal_(0).to(torch.float32)
adj_mat = torch.tensor([[0, 1, 1], [1, 0, 1], [1, 1, 0]]).to(torch.float32)
deg_mat = torch.diag(torch.sum(adj_mat, dim = 1))
deg_mat_sqrt = torch.sqrt(deg_mat)
adj_mat = torch.matmul(torch.matmul(deg_mat_sqrt.inverse(), adj_mat), deg_mat_sqrt.inverse())
edge_index = torch.nonzero(adj_mat).t()

gnn.eval()
out = gnn(features, edge_index)

gnn_state_dict = gnn.state_dict()

x = features
for l in range(nlayers):
    lin0 = gnn_state_dict[f'graph_conv_{l}.lins.0.weight']
    bias = gnn_state_dict[f'graph_conv_{l}.bias']
    z = torch.matmul(x, lin0.t())

    for k in range(K):
        x = torch.matmul(adj_mat, x)
        lin = gnn_state_dict[f'graph_conv_{l}.lins.{k+1}.weight']
        z = z + torch.matmul(x, lin.t())

    z = z + bias
    x = torch.relu(z)

print(f'out: {out}')
print(f'x: {x}')

# Check if the output is the same
print(torch.allclose(out, x))




print(adj_mat)
print(edge_index)

features = features.to(device)
edge_index = edge_index.to(device)
print(f'features: {features}')

# Load state dict

lin0_d = gnn_state_dict['graph_conv_0.lins.0.weight']
lin0_d = lin0_d.to(device)
out_lin0_d = torch.matmul(features, lin0_d.t())
print(f'out_lin0_d: {out_lin0_d.shape}')
print(f'out_lin0_d: {out_lin0_d}')

z1 = torch.matmul(adj_mat, features)
print(f'z1: {z1.shape}')
print(f'z1: {z1}')

lin1_d = gnn_state_dict['graph_conv_0.lins.1.weight']
lin1_d = lin1_d.to(device)
out_lin1_d = out_lin0_d + torch.matmul(z1, lin1_d.t())
print(out_lin1_d.shape)
print(f'out_lin1_d: {out_lin1_d}')

# adj_mat = torch.matmul(adj_mat, adj_mat)
z2 = torch.matmul(adj_mat, z1)
print(f'z2: {z2}')

lin2_d = gnn_state_dict['graph_conv_0.lins.2.weight']
lin2_d = lin2_d.to(device)
out_lin2_d = out_lin1_d + torch.matmul(z2, lin2_d.t())
print(f'out_lin2_d: {out_lin2_d}')

bias = gnn_state_dict['graph_conv_0.bias']
bias = bias.to(device)
print(f'bias: {bias.shape}')

out = out_lin2_d + bias
print(f'out: {out.shape}')

graph_conv_0 = gnn.graph_conv_0
out_true = graph_conv_0(features, edge_index)
print(f'out_true: {out_true.shape}')

# Check if the output is the same
print(torch.allclose(out, out_true))
print(torch.allclose(out[0], out_true[0]))
print(torch.allclose(out[1], out_true[1]))
print(torch.allclose(out[2], out_true[2]))

print(f'out: {out[0][0]}')
print(f'out_true: {out_true[0][0]}')

