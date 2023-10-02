import io
import torch

def save_tensor(device, my_tensor, filename):
    # print("[python] my_tensor: ", my_tensor)
    f = io.BytesIO()
    torch.save(my_tensor, f, _use_new_zipfile_serialization=True)
    with open(filename, "wb") as out_f:
        # Copy the BytesIO stream to the output file
        out_f.write(f.getbuffer())

model = torch.load('model_k3_1024_2l.pt')
print("[python] model: ", model)
gnn = model.gnn_backbone

for param in model.parameters():
    param.requires_grad = False
nlayers = 2
K = 3
num_nodes = 10
nfeatures = 34
latent_size = 256

gnn_state_dict = gnn.state_dict()
for l in range(nlayers):
    print(f'layer: {l}')
    lin = gnn_state_dict[f'graph_conv_{l}.lins.0.weight']
    bias = gnn_state_dict[f'graph_conv_{l}.bias']
    save_tensor('cpu', lin, f'k3_params/py/lin_{l}_{0}.pt')
    save_tensor('cpu', bias, f'k3_params/py/bias_{l}.pt')

    for k in range(K):
        print(f'k: {k}')
        lin = gnn_state_dict[f'graph_conv_{l}.lins.{k+1}.weight']
        save_tensor('cpu', lin, f'k3_params/py/lin_{l}_{k+1}.pt')

mlp_layer = model.gnn_mlp
mlp_layer.to('cpu')
mlp_layer.eval()

mlp_layer_state_dict = mlp_layer.state_dict()
print(mlp_layer_state_dict.keys())

l0_wts = mlp_layer_state_dict['lins.0.weight']
l0_bias = mlp_layer_state_dict['lins.0.bias']
save_tensor('cpu', l0_wts, 'k3_params/py/mlp_lin_0.pt')
save_tensor('cpu', l0_bias, 'k3_params/py/mlp_bias_0.pt')

print("============================")
print(mlp_layer.norms[0].module.state_dict().keys())
print("============================")
n0_wts = mlp_layer_state_dict['norms.0.module.weight']
n0_bias = mlp_layer_state_dict['norms.0.module.bias']
running_mean = mlp_layer_state_dict['norms.0.module.running_mean']
running_var = mlp_layer_state_dict['norms.0.module.running_var']
num_batches_tracked = mlp_layer_state_dict['norms.0.module.num_batches_tracked']
save_tensor('cpu', n0_wts, 'k3_params/py/mlp_norm_0_weight.pt')
save_tensor('cpu', n0_bias, 'k3_params/py/mlp_norm_0_bias.pt')
save_tensor('cpu', running_mean, 'k3_params/py/mlp_norm_0_running_mean.pt')
save_tensor('cpu', running_var, 'k3_params/py/mlp_norm_0_running_var.pt')
save_tensor('cpu', num_batches_tracked, 'k3_params/py/mlp_norm_0_num_batches_tracked.pt')

l1_wts = mlp_layer_state_dict['lins.1.weight']
l1_bias = mlp_layer_state_dict['lins.1.bias']

save_tensor('cpu', l1_wts, 'k3_params/py/mlp_lin_1.pt')
save_tensor('cpu', l1_bias, 'k3_params/py/mlp_bias_1.pt')

output_layer = model.output_linear
output_layer.to('cpu')
output_layer.eval()
output_layer_state_dict = output_layer.state_dict()
outlayer_wts = output_layer_state_dict['weight']
outlayer_bias = output_layer_state_dict['bias']

save_tensor('cpu', outlayer_wts, 'k3_params/py/outlayer_wts.pt')
save_tensor('cpu', outlayer_bias, 'k3_params/py/outlayer_bias.pt')

save_tensor('cpu', model.actions_mean, 'k3_params/py/actions_mean.pt')
save_tensor('cpu', model.actions_std, 'k3_params/py/actions_std.pt')
