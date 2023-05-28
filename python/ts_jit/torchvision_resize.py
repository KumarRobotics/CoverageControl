import os
import sys
import torch
import torchvision
import torchvision.transforms as T

class Resizer(torch.nn.Module):
    def __init__(self, size):
        super(Resizer, self).__init__()
        self.size = size
        self.T = T.Resize(size, interpolation=T.InterpolationMode.BILINEAR, antialias=True)

    def forward(self, img):
        return self.T(img)

if __name__ == "__main__":
    # Get size from args
    map_size = int(sys.argv[1])
    file_name = str(sys.argv[2])
    scripted_resizer = torch.jit.script(Resizer([map_size,]))
    scripted_resizer.save(file_name)
