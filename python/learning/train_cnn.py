import os
import sys
import torch

import src.data_loaders.data_loader_utils as dl_utils
from src.data_loaders.data_loaders import LocalMapCNNDataset
from src.models.cnn import CNN
from src.trainers.trainer import TrainModel

# Set the device
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

config_file = sys.argv[1]
config = dl_utils.LoadYaml(config_file)
dataset_path = config["DataDir"]
data_dir = dataset_path + "/data/"
model_dir = dataset_path + "/models/"
if not os.path.exists(model_dir):
    os.makedirs(model_dir)

model_file = model_dir + "/model.pt"


training_config = config["CNNTraining"]
batch_size = training_config["BatchSize"]
num_epochs = training_config["NumEpochs"]
learning_rate = training_config["LearningRate"]
momentum = training_config["Momentum"]
weight_decay = training_config["WeightDecay"]


cnn_config = config["CNN"]
use_comm_map = cnn_config["UseCommMap"]
output_dim = cnn_config["OutputDim"]

model = CNN(cnn_config).to(device)

pretrained_model = config["CNNModel"]["Dir"] + config["CNNModel"]["Model"]
if pretrained_model is not None:
    model.LoadModel(pretrained_model)
    print("Loaded pretrained model: {}".format(pretrained_model))

train_dataset = LocalMapCNNDataset(data_dir, "train", use_comm_map, output_dim)
val_dataset = LocalMapCNNDataset(data_dir, "val", use_comm_map, output_dim)
test_dataset = LocalMapCNNDataset(data_dir, "test", use_comm_map, output_dim)

model.register_buffer("coverage_features_mean", train_dataset.targets_mean)
model.register_buffer("coverage_features_std", train_dataset.targets_std)

print("Loaded datasets")
print("Train dataset size: {}".format(len(train_dataset)))

train_loader = torch.utils.data.DataLoader(train_dataset, batch_size=batch_size, shuffle=True)
val_loader = torch.utils.data.DataLoader(val_dataset, batch_size=batch_size, shuffle=True)
test_loader = torch.utils.data.DataLoader(test_dataset, batch_size=batch_size, shuffle=True)

optimizer = torch.optim.SGD(model.parameters(), lr=learning_rate, momentum=momentum, weight_decay=weight_decay)

# Use mse loss for regression
criterion = torch.nn.MSELoss()

trainer = TrainModel(model, train_loader, val_loader, test_loader, optimizer, criterion, num_epochs, device, model_file)

trainer.Train()
