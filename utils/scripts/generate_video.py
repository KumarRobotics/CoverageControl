import torch
import torchvision

"""
This is just some helpful script for debugging the dataset generation
"""
T = torchvision.transforms.ConvertImageDtype(torch.uint8)

# Load data from jit
local_maps = list(torch.load('local_maps.pt').parameters())[0]
print(local_maps.shape)
obstacle_maps = list(torch.load('obstacle_maps.pt').parameters())[0].to_dense()
print(obstacle_maps.shape)
comm_maps = list(torch.load('comm_maps.pt').parameters())[0].to_dense()
print(comm_maps.shape)

local_maps_0 = local_maps[:, 0, :, :]
obstacle_maps_0 = obstacle_maps[:, 0, :, :]
comm_maps_0 = comm_maps[:, 0, 0, :, :]

# Check if obstacle maps are all zeros
print(torch.sum(obstacle_maps_0))

local_maps_0 = T.forward(local_maps_0).unsqueeze(-1)
obstacle_maps_0 = T.forward(obstacle_maps_0).unsqueeze(-1)
comm_maps_0 = T.forward(comm_maps_0).unsqueeze(-1)
comm_maps_0[comm_maps_0 > 0] = 255
print(local_maps_0.shape)
print(obstacle_maps_0.shape)
print(comm_maps_0.shape)


maps = torch.stack([0.5*obstacle_maps_0, comm_maps_0, local_maps_0], dim=3)
maps.shape


print(maps.shape)

# Generate video
torchvision.io.write_video('maps.mp4', maps[:,:,:,:,0], fps=10)
