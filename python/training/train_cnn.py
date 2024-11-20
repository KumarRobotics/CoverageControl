import os
import pathlib
import sys

import torch
from coverage_control import IOUtils
from coverage_control.nn import CNN
from coverage_control.nn import LocalMapCNNDataset
from coverage_control.nn import TrainModel

# Set the device
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

config_file = sys.argv[1]
config = IOUtils.load_toml(config_file)
num_workers = config["NumWorkers"]
dataset_path = pathlib.Path(IOUtils.sanitize_path(config["DataDir"]))
data_dir = dataset_path / "data/"

cnn_model = config["CNNModel"]
model_dir = IOUtils.sanitize_path(cnn_model["Dir"]) + "/"

if not os.path.exists(model_dir):
    os.makedirs(model_dir)

training_config = config["CNNTraining"]
batch_size = training_config["BatchSize"]
num_epochs = training_config["NumEpochs"]
learning_rate = training_config["LearningRate"]
momentum = training_config["Momentum"]
weight_decay = training_config["WeightDecay"]
output_dim = config["CNNBackBone"]["OutputDim"]

use_comm_map = config["ModelConfig"]["UseCommMaps"]
cnn_config = config["CNNBackBone"]

model = CNN(cnn_config).to(device)

train_dataset = LocalMapCNNDataset(str(data_dir), "train", use_comm_map, output_dim)
val_dataset = LocalMapCNNDataset(str(data_dir), "val", use_comm_map, output_dim)

model.register_buffer("target_mean", train_dataset.targets_mean)
model.register_buffer("target_Std", train_dataset.targets_std)

print("Loaded datasets")
print(f"Train dataset size: {len(train_dataset)}")

train_loader = torch.utils.data.DataLoader(
        train_dataset, batch_size=batch_size, shuffle=True, num_workers=num_workers
        )
val_loader = torch.utils.data.DataLoader(
        val_dataset, batch_size=batch_size, shuffle=False, num_workers=num_workers
        )

optimizer = torch.optim.SGD(
        model.parameters(), lr=learning_rate, momentum=momentum, weight_decay=weight_decay
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
# trainer.LoadSavedModel(model_dir)
# trainer.LoadSavedOptimizer(optimizer_file)

trainer.train()

test_dataset = LocalMapCNNDataset(str(data_dir), "test", use_comm_map, output_dim)
test_loader = torch.utils.data.DataLoader(
        test_dataset, batch_size=batch_size, shuffle=False, num_workers=num_workers
        )
test_loss = trainer.test(test_loader)
