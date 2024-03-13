import torch
from tagconv1 import TAGConv1
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
torch.save(gnn.state_dict(), 'gnn_state_dict.pt')
print(gnn.graph_conv_0)
print(gnn.graph_conv_0.state_dict().keys())

# Create a feature input tensor for num_nodes nodes with 34 features
features = torch.randn(num_nodes, nfeatures)
# Create a binary graph adjacency matrix for num_nodes nodes
adj_mat = torch.randint(0, 2, (num_nodes, num_nodes))
adj_mat = adj_mat + adj_mat.t()
adj_mat[adj_mat > 0] = 1
adj_mat = adj_mat.fill_diagonal_(0).to(torch.float32)
# adj_mat = torch.tensor([[0, 0, 1], [0, 0, 1], [1, 1, 0]]).to(torch.float32)

deg_mat = torch.diag(torch.sum(adj_mat, dim = 1))
deg_mat_sqrt = torch.sqrt(deg_mat)
shape_mat = torch.matmul(torch.matmul(deg_mat_sqrt.inverse(), adj_mat), deg_mat_sqrt.inverse())

edge_index = torch.nonzero(shape_mat).t()
print(adj_mat)
print(shape_mat)
print(edge_index)


# print(features)
# print(edge_weights)

gnn.eval()
features = features.to(device)
edge_index = edge_index.to(device)

gnn_out = gnn(features, edge_index)

gnn_state_dict = gnn.state_dict()

x = features
for l in range(nlayers):
    print(f'layer: {l}')
    lin0 = gnn_state_dict[f'graph_conv_{l}.lins.0.weight']
    print(f'lin0 shape: {lin0.shape}')
    bias = gnn_state_dict[f'graph_conv_{l}.bias']
    print(f'bias shape: {bias.shape}')
    print(f'x shape: {x.shape}')
    z = torch.matmul(x, lin0.t())
    print(f'z shape: {z.shape}')

    for k in range(K):
        print(f'k: {k}')
        x = torch.matmul(shape_mat, x)
        print(f'x shape: {x.shape}')
        lin = gnn_state_dict[f'graph_conv_{l}.lins.{k+1}.weight']
        z = z + torch.matmul(x, lin.t())
        # print(f'z: {z}')
        print(f'z shape: {z.shape}')

    z = z + bias

    print(f'z shape: {z.shape}')
    # print(f'z: {z}')
    x = torch.relu(z)
    print(f'x shape: {x.shape}')

# Check if the output is the same
print(torch.allclose(gnn_out, x))
print(x)


x = features
print('==================')

X = list()
Y = list()
Z = list()

for l in range(0, nlayers + 1):
    if l == 0:
        Y.append(torch.zeros(num_nodes, nfeatures))
    elif l == 1:
        Y.append(torch.zeros(num_nodes, K + 1, nfeatures))
    else:
        Y.append(torch.zeros(num_nodes, K + 1, latent_size))

for l in range(0, nlayers + 1):
    if l == 0:
        X.append(torch.zeros(num_nodes, nfeatures))
        Z.append(torch.zeros(num_nodes, nfeatures))
    else:
        X.append(torch.zeros(num_nodes, latent_size))
        Z.append(torch.zeros(num_nodes, latent_size))

for i in range(0, num_nodes):
    X[0][i] = features[i]
    Z[0][i] = features[i]
    Y[0][i] = features[i]
for l in range(1, nlayers + 1):
    print(f'layer: {l}')
    for k in range(0, K + 1):
        print(f'k: {k}')
        for i in range(0, num_nodes):

            if k == 0:
                Y[l][i, 0] = X[l-1][i]
                Z[l][i] = torch.matmul(Y[l][i, 0], gnn_state_dict[f'graph_conv_{l-1}.lins.0.weight'].t())
                continue
            deg_i = torch.sum(adj_mat[i, :])
            for j in range(0, num_nodes):
                if i == j:
                    continue
                if adj_mat[i, j] == 0:
                    continue
                deg_j = torch.sum(adj_mat[j, :])
                Y[l][i, k] = Y[l][i, k] + Y[l][j, k - 1] / torch.sqrt(deg_i * deg_j)
            Z[l][i] = Z[l][i] + torch.matmul(Y[l][i, k], gnn_state_dict[f'graph_conv_{l-1}.lins.{k}.weight'].t())

    for i in range(0, num_nodes):
        Z[l][i] = Z[l][i] + gnn_state_dict[f'graph_conv_{l-1}.bias']
        X[l][i] = torch.relu(Z[l][i])
    # print(f'X[l]: {X[l]}')

out = X[-1]

print(torch.allclose(gnn_out, out))

# Get number of non zero elements
# print(torch.nonzero(X[-1]).shape)
# print(torch.nonzero(gnn_out).shape)
# print(Z)
