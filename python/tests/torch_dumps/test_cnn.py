import io
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
nlayers = 2
K = 3
num_nodes = 10
nfeatures = 34
latent_size = 256

model = torch.load('model_k3_1024_2l.pt')
model.eval()
model.to(device)

cnn = model.cnn_backbone
print(cnn)
cnn.eval()
# torch.jit.save(gnn, 'cnn_k3_1024_jit.pt')
print(cnn.batch_norm0)
# Get the state dict keys
print(cnn.conv0.state_dict())
print(cnn.batch_norm0.state_dict())
print(cnn.state_dict().keys())

dataset_dir = '/root/CoverageControl_ws/datasets/1024/data/val/'
maps = dl_utils.LoadMaps(dataset_dir, True)
maps = maps[50:110]
print('maps.shape: ', maps.shape)

num_channels = maps.shape[2]
image_size = maps.shape[3]

in_maps = maps.view(-1, num_channels, image_size, image_size)
dataset_size = maps.shape[0]

print('in_maps.shape: ', in_maps.shape)

cnn_output = cnn(in_maps.view(-1, in_maps.shape[-3], in_maps.shape[-2], in_maps.shape[-1]))

cnn_output1 = cnn(in_maps.view(-1, in_maps.shape[-3], in_maps.shape[-2], in_maps.shape[-1]))
print(torch.all(torch.eq(cnn_output, cnn_output1)))
    

print('cnn_output.shape: ', cnn_output.shape)

traced_script_module = torch.jit.trace(cnn, maps[50])
traced_script_module.save("cnn_jit.pt")

sample_input = maps.clone().detach()
def save_tensor(device, my_tensor, filename):
    print("[python] my_tensor: ", my_tensor)
    f = io.BytesIO()
    torch.save(my_tensor, f, _use_new_zipfile_serialization=True)
    with open(filename, "wb") as out_f:
        # Copy the BytesIO stream to the output file
        out_f.write(f.getbuffer())

# save_tensor('cpu', sample_input, 'sample_input_zip.pt')

cnn_output2 = list(torch.jit.load('out.pt').parameters())[0]
print(torch.all(torch.eq(cnn_output, cnn_output2)))
print(torch.sum(torch.abs(cnn_output - cnn_output2)))
print(torch.sum(torch.abs(cnn_output)))
# Save sample input to be read by C++ code
# torch.jit.save(sample_input, 'sample_input.pt')
# f = io.BytesIO()
# torch.save(sample_input, f, _use_new_zipfile_serialization=True)
# # Save to file
# with open('sample_input.pickle', 'wb') as file:
#     file.write(f.getvalue())


# # gnn = model
# # gnn = model.gnn_backbone
# # print(gnn)
# torch.save(gnn.state_dict(), 'gnn_state_dict.pt')
# print(gnn.graph_conv_0)
# print(gnn.graph_conv_0.state_dict().keys())


class Container(torch.nn.Module):
    def __init__(self, my_values):
        super().__init__()
        for key in my_values:
            setattr(self, key, my_values[key])

my_values = {
    'tensor': sample_input
}

# Save arbitrary values supported by TorchScript
# https://pytorch.org/docs/master/jit.html#supported-type
# container = torch.jit.script(Container(my_values))
# container.save("sample_input_container.pt")
