import torch
import pyCoverageControlTorch as cct

arr = torch.tensor([[1, 2, 3], [4, 5, 6]], dtype=torch.float32, device='cuda')
print(arr)
print(cct.d_sigmoid(arr))
