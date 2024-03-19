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

"""
Train a model using pytorch
"""

import time

import torch

__all__ = ["TrainModel"]


## @ingroup python_api
class TrainModel:
    """
    Train a model using pytorch

    """

    def __init__(
        self,
        model: torch.nn.Module,
        train_loader: torch.utils.data.DataLoader,
        val_loader: torch.utils.data.DataLoader,
        optimizer: torch.optim.Optimizer,
        criterion: torch.nn.Module,
        epochs: int,
        device: torch.device,
        model_file: str,
        optimizer_file: str,
    ):
        """
        Initialize the model trainer

        Args:
            model: torch.nn.Module
            train_loader: loader for the training data
            val_loader: loader for the validation data
            optimizer: optimizer for the model
            criterion: loss function
            epochs: number of epochs
            device: device to train the model
            model_file: file to save the model
            optimizer_file: file to save the optimizer
        """
        self.model = model
        self.train_loader = train_loader
        self.val_loader = val_loader
        self.optimizer = optimizer
        self.criterion = criterion
        self.epochs = epochs
        self.device = device
        self.model_file = model_file
        self.optimizer_file = optimizer_file

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
        """
        # Initialize the best validation loss
        best_val_loss = float("Inf")
        best_train_loss = float("Inf")

        # Initialize the loss history
        train_loss_history = []
        val_loss_history = []
        start_time = time.time()

        model_path = self.model_file.split(".")[0]
        # Train the model

        for epoch in range(self.epochs):
            # Training
            train_loss = self.TrainEpoch()
            train_loss_history.append(train_loss)
            torch.save(train_loss_history, model_path + "_train_loss.pt")
            # Print the loss
            print(
                "Epoch: {}/{}.. ".format(epoch + 1, self.epochs),
                "Training Loss: {:.5f}.. ".format(train_loss),
            )

            # Validation

            if self.val_loader is not None:
                val_loss = self.validate_epoch(self.val_loader)
                val_loss_history.append(val_loss)
                torch.save(val_loss_history, model_path + "_val_loss.pt")

                # Save the best model

                if val_loss < best_val_loss:
                    best_val_loss = val_loss
                    torch.save(self.model, self.model_file)
                    torch.save(self.optimizer, self.optimizer_file)
                print(
                    "Epoch: {}/{}.. ".format(epoch + 1, self.epochs),
                    "Validation Loss: {:.5f}.. ".format(val_loss),
                    "Best Validation Loss: {:.5f}.. ".format(best_val_loss),
                )

            if train_loss < best_train_loss:
                best_train_loss = train_loss
                torch.save(self.model, model_path + "_curr.pt")
                torch.save(self.optimizer, model_path + "_optimizer_curr.pt")

            if epoch % 5 == 0:
                torch.save(self.model, model_path + "_epoch" + str(epoch) + ".pt")

            elapsed_time = time.time() - start_time
            # Print elapsed time in minutes
            print("Elapsed time: {:.2f} minutes".format(elapsed_time / 60))

    # Train the model in batches
    def TrainEpoch(self) -> float:
        """
        Train the model in batches

        Returns:
            training loss
        """
        # Initialize the training loss
        train_loss = 0.0

        # Set the model to training mode
        self.model.train()

        num_dataset = 0
        # Train the model in batches

        for batch_idx, (data, target) in enumerate(self.train_loader):
            # Move the data to the device
            data, target = data.to(self.device), target.to(self.device)

            if target.dim() == 3:
                target = target.view(-1, target.shape[-1])

            # Clear the gradients
            self.optimizer.zero_grad()

            # Forward propagation
            output = self.model(data)

            # Calculate the loss
            loss = self.criterion(output, target)

            # Print batch number and loss

            if batch_idx % 10 == 0:
                print("Batch: {}, Loss: {}".format(batch_idx, loss))

            # Backward propagation
            loss.backward()

            # Update the parameters
            self.optimizer.step()

            # Update the training loss
            # train_loss += loss.item() * data.size(0)
            # num_dataset += data.size(0)
            train_loss += loss.item()
            num_dataset += 1

        # Return the training loss

        return train_loss / num_dataset

    # Validate the model in batches
    def validate_epoch(self, data_loader: torch.utils.data.DataLoader) -> float:
        """
        Validate the model in batches

        Args:
            data_loader: data loader for the validation data

        Returns:
            validation loss
        """
        # Initialize the validation loss
        val_loss = 0.0

        # Set the model to evaluation mode
        self.model.eval()

        num_dataset = 0
        # Validate the model in batches
        with torch.no_grad():
            for batch_idx, (data, target) in enumerate(self.val_loader):
                # Move the data to the device
                data, target = data.to(self.device), target.to(self.device)

                if target.dim() == 3:
                    target = target.view(-1, target.shape[-1])

                # Forward propagation
                output = self.model(data)

                # Calculate the loss
                loss = self.criterion(output, target)

                # Update the validation loss
                # val_loss += loss.item() * data.size(0)
                # num_dataset += data.size(0)
                val_loss += loss.item()
                num_dataset += 1

        # Return the validation loss

        return val_loss / num_dataset

    # Test the model in batches
    def Test(self, test_loader: torch.utils.data.DataLoader) -> float:
        """
        Test the model in batches

        Args:
            test_loader: data loader for the test data

        Returns:
            test loss
        """

        return self.validate_epoch(test_loader)
