class CNNConfigParser():
    def __init__(self):
        pass

    def Parse(self, config):
        self.config = config
        self.input_dim = self.config['InputDim']
        self.output_dim = self.config['OutputDim']
        self.num_layers = self.config['NumLayers']
        self.latent_size = self.config['LatentSize']
        self.kernel_size = self.config['KernelSize']
        self.image_size = self.config['ImageSize']
        self.backbone_output_dim = self.config['BackBoneOutputDim']

class GNNConfigParser():
    def __init__(self, config):
        pass

    def Parse(self, config):
        self.config = config
        self.input_dim = self.config['InputDim']
        self.output_dim = self.config['OutputDim']
        self.num_hops = self.config['NumHops']
        self.num_layers = self.config['NumLayers']
        self.latent_size = self.config['LatentSize']

