"""
Train the LPAC model
"""

# @file train_lpac.py
#  @brief Train the LPAC model
import os
import pathlib
import sys

import torch
import torch_geometric
from coverage_control import IOUtils
from coverage_control.nn import CNNGNNDataset
from coverage_control.nn import LPAC
from coverage_control.nn import TrainModel

# Set the device
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

config_file = sys.argv[1]
world_size = int(sys.argv[2])
config = IOUtils.load_toml(config_file)
num_workers = config["NumWorkers"]
dataset_path = pathlib.Path(IOUtils.sanitize_path(config["DataDir"]))
data_dir = dataset_path / "data/"

lpac_model = config["LPACModel"]
model_dir = IOUtils.sanitize_path(lpac_model["Dir"]) + "/"

if not os.path.exists(model_dir):
    os.makedirs(model_dir)

training_config = config["LPACTraining"]
batch_size = training_config["BatchSize"]
num_epochs = training_config["NumEpochs"]
learning_rate = training_config["LearningRate"]
# momentum = training_config["Momentum"]
weight_decay = training_config["WeightDecay"]

use_comm_map = config["ModelConfig"]["UseCommMaps"]

model = LPAC(config).to(device)
# model = torch.compile(model, dynamic=True)

# cnn_pretrained_model = config["CNNModel"]["Dir"] + config["CNNModel"]["Model"]
# model.LoadCNNBackBone(cnn_pretrained_model)

if "PreTrainedModel" in config["LPACModel"]:
    lpac_pretrained_model = (
            IOUtils.sanitize_path(config["LPACModel"]["Dir"])
            + "/" + config["LPACModel"]["PreTrainedModel"]
            )
    model.load_model(lpac_pretrained_model)

train_dataset = CNNGNNDataset(data_dir, "train", use_comm_map, world_size)
val_dataset = CNNGNNDataset(data_dir, "val", use_comm_map, world_size)

# Check if buffer exists

if not hasattr(model, "actions_mean"):
    model.register_buffer("actions_mean", train_dataset.targets_mean.to(device))
    model.register_buffer("actions_std", train_dataset.targets_std.to(device))
else:
    model.actions_mean = train_dataset.targets_mean
    model.actions_std = train_dataset.targets_std

print("Loaded datasets")
print(f"Train dataset size: {len(train_dataset)}")

train_loader = torch_geometric.loader.DataLoader(
        train_dataset, batch_size=batch_size, shuffle=True, num_workers=num_workers
        )
val_loader = torch_geometric.loader.DataLoader(
        val_dataset, batch_size=batch_size, shuffle=False, num_workers=num_workers
        )

# optimizer = torch.optim.SGD(
#         model.parameters(),
#         lr=learning_rate,
#         momentum=momentum,
#         weight_decay=weight_decay
#         )
optimizer = torch.optim.Adam(
        model.parameters(),
        lr=learning_rate,
        weight_decay=weight_decay
        )

# Use mse loss for regression
criterion = torch.nn.MSELoss()

trainer = TrainModel(
        model,
        train_loader,
        val_loader,
        optimizer,
        criterion,
        num_epochs,
        device,
        model_dir,
        )
# trainer = TrainModel(model, train_loader, val_loader, optimizer, criterion, num_epochs, device, model_dir)

trainer.train()

test_dataset = CNNGNNDataset(data_dir, "test", use_comm_map, world_size)
test_loader = torch_geometric.loader.DataLoader(
        test_dataset, batch_size=batch_size, shuffle=False, num_workers=num_workers
        )
test_loss = trainer.test(test_loader)
