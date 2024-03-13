import sys
import torch

import coverage_control

if __name__ == "__main__":
    # Get filename from argument
    filename = sys.argv[1]
    # Load tensor from file
    tensor_model = torch.jit.load(filename)
    tensor = list(tensor_model.parameters())[0]
    # Print tensor
    print(tensor.dtype)
    print(tensor.shape)
    print(tensor.sum())


