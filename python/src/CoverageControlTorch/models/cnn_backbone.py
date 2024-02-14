import torch
import math
import tomllib
from .config_parser import CNNConfigParser

class CNNBackBone(torch.nn.Module, CNNConfigParser):
    """
    Implements a multi-layer convolutional neural network, with ReLU non-linearities between layers,
    according to hyperparameters specified in the config
    """
    def __init__(self, config):
        super(CNNBackBone, self).__init__()
        self.Parse(config)

        self.add_module("conv0", torch.nn.Conv2d(self.input_dim, self.latent_size, kernel_size=self.kernel_size))
        self.add_module("batch_norm0", torch.nn.BatchNorm2d(self.latent_size))
        for layer in range(self.num_layers - 1):
            self.add_module("conv{}".format(layer + 1), torch.nn.Conv2d(self.latent_size, self.latent_size, kernel_size=self.kernel_size))
            self.add_module("batch_norm{}".format(layer + 1), torch.nn.BatchNorm2d(self.latent_size))

        self.flatten_size = self.latent_size * (self.image_size - self.num_layers * (self.kernel_size - 1)) ** 2

        self.add_module("linear_1", torch.nn.Linear(self.flatten_size, self.latent_size))
        # self.add_module("linear_2", torch.nn.Linear(self.latent_size, self.backbone_output_dim))
        # self.add_module("linear_3", torch.nn.Linear(2 * self.output_dim, self.output_dim))


    def forward(self, x):
        for layer in range(self.num_layers):
            x = torch.nn.functional.leaky_relu(self._modules["batch_norm{}".format(layer)](self._modules["conv{}".format(layer)](x)))
            # x = self._modules["conv{}".format(layer)](x)
            # x = self._modules["batch_norm{}".format(layer)](x)
            # x = torch.nn.functional.leaky_relu(x)

        x = x.flatten(1)
        x = torch.nn.functional.leaky_relu(self.linear_1(x))
        return x
        # x = torch.nn.functional.leaky_relu(self.linear_2(x))
        # x = self.linear_3(x)
        # output = x.reshape(x.shape[0], self.latent_size, -1)
        # output, _ = torch.max(output, dim=2)
        # return output
