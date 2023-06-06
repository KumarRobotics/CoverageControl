import os
import sys
import torch
import torch_geometric

import src.data_loaders.data_loader_utils as dl_utils
from src.data_loaders.data_loaders import LocalMapGNNDataset
from src.models.gnn import CNNGNN
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

model_dir = config["ModelDir"]
if not os.path.exists(model_dir):
    os.makedirs(model_dir)

model_file = model_dir + config["ModelFile"]

training_config = config["GNNTraining"]
batch_size = training_config["BatchSize"]
num_epochs = training_config["NumEpochs"]
learning_rate = training_config["LearningRate"]
momentum = training_config["Momentum"]
weight_decay = training_config["WeightDecay"]

use_comm_map = config["GNN"]["UseCommMap"]

model = CNNGNN(config).to(device)


cnn_pretrained_model = config["CNNModel"]["Dir"] + config["CNNModel"]["Model"]
if cnn_pretrained_model is not None:
    model.LoadCNNBackBone(cnn_pretrained_model)
    print("Loaded pretrained model: {}".format(cnn_pretrained_model))

train_dataset = LocalMapGNNDataset(data_dir, "train", use_comm_map)
val_dataset = LocalMapGNNDataset(data_dir, "val", use_comm_map)
test_dataset = LocalMapGNNDataset(data_dir, "test", use_comm_map)

model.register_buffer("actions_mean", train_dataset.targets_mean)
model.register_buffer("actions_std", train_dataset.targets_std)

print("Loaded datasets")
print("Train dataset size: {}".format(len(train_dataset)))

train_loader = torch_geometric.loader.DataLoader(train_dataset, batch_size=batch_size, shuffle=True)
val_loader = torch_geometric.loader.DataLoader(val_dataset, batch_size=batch_size, shuffle=True)
test_loader = torch_geometric.loader.DataLoader(test_dataset, batch_size=batch_size, shuffle=True)

optimizer = torch.optim.SGD(model.parameters(), lr=learning_rate, momentum=momentum, weight_decay=weight_decay)

# Use mse loss for regression
criterion = torch.nn.MSELoss()

trainer = TrainModel(model, train_loader, val_loader, test_loader, optimizer, criterion, num_epochs, device, model_file)

trainer.Train()
