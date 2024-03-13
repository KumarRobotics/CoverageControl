#  This file is part of the CoverageControl library
#
#  Author: Saurav Agarwal
#  Contact: sauravag@seas.upenn.edu, agr.saurav1@gmail.com
#  Repository: https://github.com/KumarRobotics/CoverageControl
#
#  Copyright (c) 2024, Saurav Agarwal
#
#  The CoverageControl library is free software: you can redistribute it and/or
#  modify it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or (at your
#  option) any later version.
#
#  The CoverageControl library is distributed in the hope that it will be
#  useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
#  Public License for more details.
#
#  You should have received a copy of the GNU General Public License along with
#  CoverageControl library. If not, see <https://www.gnu.org/licenses/>.

import torch

__all__ = ["MultiTrainModel"]
class MultiTrainModel():
    """
    Train a model using pytorch

    Use the class TrainModel in trainer.py to train a single model
    """

    def __init__(self, models: torch.nn.Module, train_loader: torch.utils.data.DataLoader, val_loader: torch.utils.data.DataLoader, test_loader: torch.utils.data.DataLoader, optimizers: torch.optim.Optimizer, criterion: torch.nn.Module, epochs: int, devices: list, model_files: list, optimizer_files: list):
        """
        Initialize the model trainer

        Args:
            models: list of torch.nn.Module
            train_loader: loader for the training data
            val_loader: loader for the validation data
            test_loader: loader for the test data
            optimizers: list of optimizers for the model
            criterion: loss function
            epochs: number of epochs
            devices: list of devices to train the model
            model_files: list of files to save the model
            optimizer_files: list of files to save the optimizer
        """
        self.models = models
        self.train_loader = train_loader
        self.val_loader = val_loader
        self.test_loader = test_loader
        self.optimizers = optimizers
        self.criterion = criterion
        self.epochs = epochs
        self.devices = devices
        self.model_files = model_files
        self.optimizer_files = optimizer_files
        for i, model in enumerate(self.models):
            device = torch.device('cuda', index = self.devices[i])
            self.models[i] = model.to(device)

    def load_saved_model_dict(self, model_path: str) -> None:
        """
        Load the saved model

        Args:
            model_path: model path
        """
        self.model.load_state_dict(torch.load(model_path))

    def load_saved_model(self, model_path: str) -> None:
        """
        Load the saved model

        Args:
            model_path: model path
        """
        self.model = torch.load(model_path)

    def load_saved_optimizer(self, optimizer_path: str) -> None:
        """
        Load the saved optimizer

        Args:
            optimizer_path: optimizer path
        """
        self.optimizer = torch.load(optimizer_path)

    # Train in batches, save the best model using the validation set
    def train(self) -> None:
        """
        Train the model

        Args:
            None
        """
        # Initialize the best validation loss
        best_val_losses = [float('inf')] * len(self.models)

        # Initialize the loss history
        train_loss_histories = [[] for _ in range(len(self.models))]
        val_loss_histories = [[] for _ in range(len(self.models))]

        # Train the model
        for epoch in range(self.epochs):
            # Training
            train_losses = self.TrainEpoch()
            train_loss_histories = [train_loss_history + [train_loss] for train_loss_history, train_loss in zip(train_loss_histories, train_losses)]

            # Validation
            val_losses = self.ValidateEpoch()
            val_loss_histories = [val_loss_history + [val_loss] for val_loss_history, val_loss in zip(val_loss_histories, val_losses)]

            # Save the best model
            for i, (val_loss, best_val_loss, model, model_file, optimizer, optimizer_file) in enumerate(zip(val_losses, best_val_losses, self.models, self.model_files, self.optimizers, self.optimizer_files)):
                if val_loss < best_val_loss:
                    best_val_losses[i] = val_loss
                    torch.save(model, model_file)
                    torch.save(optimizer, optimizer_file)

            # Print the loss
            print("Epoch: {}, Train Loss: {}, Val Loss: {}".format(epoch, train_losses, val_losses))

            # Save the loss history
            for i, (train_loss_history, val_loss_history) in enumerate(zip(train_loss_histories, val_loss_histories)):
                model_path = self.model_files[i].split('.')[0]
                torch.save(train_loss_history, model_path + '_train_loss.pt')
                torch.save(val_loss_history, model_path + '_val_loss.pt')

    # Train the model in batches
    def train_epoch(self):
        """
        Train the model in batches
        """
        # Initialize the training loss
        train_losses = [0.0] * len(self.models)

        # Set the model to training mode
        for model in self.models:
            model.train()

        num_dataset = 0
        # Train the model in batches
        for batch_idx, (data, target) in enumerate(self.train_loader):

            if target.dim() == 3:
                target = target.view(-1, target.shape[-1])

            # Clear the gradients
            for optimizer in self.optimizers:
                optimizer.zero_grad()

            # Move the data to the device
            for i in range(len(self.models)):
                data, target = data.to(torch.device('cuda', index=self.devices[i])), target.to(torch.device('cuda', index=self.devices[i]))

                # Forward propagation
                output = self.models[i](data)

                # Calculate the loss
                loss = self.criterion(output, target)

                # Print batch number and loss
                if batch_idx % 100 == 0:
                    print("i: {}, Batch: {}, Loss: {}".format(i, batch_idx, loss.item()))

                # Backward propagation
                loss.backward()
                self.optimizers[i].step()

                # Update the training loss
                train_losses[i] += loss.item() * data.size(0)
            num_dataset += data.size(0)

        # Return the training loss
        return train_losses/num_dataset

    # Validate the model in batches
    def validate_epoch(self):
        """
        Validate the model in batches
        """
        # Initialize the validation loss
        val_losses = [0.0] * len(self.models)

        # Set the model to evaluation mode
        for model in self.models:
            model.eval()

        num_dataset = 0
        # Validate the model in batches
        with torch.no_grad():
            for batch_idx, (data, target) in enumerate(self.val_loader):
                if target.dim() == 3:
                    target = target.view(-1, target.shape[-1])

                # Move the data to the device
                for i in range(self.devices):
                    device = torch.device('cuda', index=self.devices[i])
                    data, target = data.to(device), target.to(device)

                    # Forward propagation
                    output = self.models[i](data)

                    # Calculate the loss
                    loss = self.criterion(output, target)

                    # Update the validation loss
                    val_losses[i] += loss.item() * data.size(0)
                num_dataset += data.size(0)

        # Return the validation loss
        return val_losses/num_dataset

    # Test the model in batches
    def test(self):
        """
        Test the model in batches
        :return: test loss
        """
        # Initialize the test loss
        test_losses = [0.0] * len(self.models)

        # Set the model to evaluation mode
        for model in self.models:
            model.eval()

        num_dataset = 0
        # Test the model in batches
        with torch.no_grad():
            for batch_idx, (data, target) in enumerate(self.test_loader):
                if target.dim() == 3:
                    target = target.view(-1, target.shape[-1])

                # Move the data to the device
                for i in range(self.devices):
                    device = torch.device('cuda', index=self.devices[i])
                    data, target = data.to(device), target.to(device)

                    # Forward propagation
                    output = self.models[i](data)

                    # Calculate the loss
                    loss = self.criterion(output, target)

                    # Update the test loss
                    test_losses[i] += loss.item() * data.size(0)
                num_dataset += data.size(0)

        # Return the test loss
        return test_losses/num_dataset
