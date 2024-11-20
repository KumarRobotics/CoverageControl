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
from copy import deepcopy

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
            num_epochs: int,
            device: torch.device,
            model_dir: str,
            ):
        """
        Initialize the model trainer

        Args:
            model: torch.nn.Module
            train_loader: loader for the training data
            val_loader: loader for the validation data
            optimizer: optimizer for the model
            criterion: loss function
            num_epochs: number of epochs
            device: device to train the model
            model_dir: dir to save the model
        """
        self.model = model
        self.train_loader = train_loader
        self.val_loader = val_loader
        self.optimizer = optimizer
        self.criterion = criterion
        self.num_epochs = num_epochs
        self.device = device
        self.model_dir = model_dir
        self.start_time = time.time()

    def load_saved_model_dict(self, model_file: str) -> None:
        """
        Load the saved model

        Args:
            model_file: model file
        """
        self.model.load_state_dict(torch.load(model_file))

    def load_saved_model(self, model_file: str) -> None:
        """
        Load the saved model

        Args:
            model_file: model path
        """
        self.model = torch.load(model_file)

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

        best_model_state_dict = None
        best_train_model_state_dict = None

        # Train the model

        for epoch in range(self.num_epochs):
            # Training
            train_loss = self.train_epoch()
            train_loss_history.append(train_loss)
            torch.save(train_loss_history, self.model_dir + "/train_loss.pt")
            # Print the loss
            print(f"Epoch: {epoch + 1}/{self.num_epochs} ",
                  f"Training Loss: {train_loss:.3e} ")

            # Validation

            if self.val_loader is not None:
                val_loss = self.validate_epoch(self.val_loader)
                val_loss_history.append(val_loss)
                torch.save(val_loss_history, self.model_dir + "/val_loss.pt")

                # Save the best model

                if val_loss < best_val_loss:
                    best_val_loss = val_loss
                    best_model_state_dict = deepcopy(self.model.state_dict())
                    # torch.save(self.model.state_dict(), self.model_dir + "/model.pt")
                    # torch.save(self.optimizer.state_dict(), self.model_dir + "/optimizer.pt")
                print(f"Epoch: {epoch + 1}/{self.num_epochs} ",
                      f"Validation Loss: {val_loss:.3e} ",
                      f"Best Validation Loss: {best_val_loss:.3e}")

            if train_loss < best_train_loss:
                best_train_loss = train_loss
                best_train_model_state_dict = deepcopy(self.model.state_dict())
                # torch.save(self.model.state_dict(), self.model_dir + "/model_curr.pt")
                # torch.save(self.optimizer.state_dict(), self.model_dir + "/optimizer_curr.pt")

            if epoch % 5 == 0:
                torch.save({"epoch": epoch,
                            "model_state_dict": self.model.state_dict(),
                            "optimizer_state_dict": self.optimizer.state_dict(),
                            "loss": train_loss},
                           self.model_dir + "/model_epoch" + str(epoch) + ".pt")

            torch.save(best_model_state_dict, self.model_dir + "/model.pt")
            torch.save(best_train_model_state_dict, self.model_dir + "/model_train.pt")
            elapsed_time = time.time() - start_time
            # Print elapsed time in minutes
            print(f"Elapsed time: {elapsed_time / 60:.2f} minutes")

    # Train the model in batches
    def train_epoch(self) -> float:
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
                print(f"Batch: {batch_idx}, Loss: {loss:.3e} ")

            # Backward propagation
            loss.backward()

            # Update the parameters
            self.optimizer.step()

            # Update the training loss
            train_loss += loss.item() * data.size(0)
            num_dataset += data.size(0)

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
                val_loss += loss.item() * data.size(0)
                num_dataset += data.size(0)

        # Return the validation loss

        return val_loss / num_dataset

    # Test the model in batches
    def test(self, test_loader: torch.utils.data.DataLoader) -> float:
        """
        Test the model in batches

        Args:
            test_loader: data loader for the test data

        Returns:
            test loss
        """

        test_loss = self.validate_epoch(test_loader)
        print("Test Loss: {:.3e} ".format(test_loss))
        torch.save(test_loss, self.model_dir + "/test_loss.pt")

