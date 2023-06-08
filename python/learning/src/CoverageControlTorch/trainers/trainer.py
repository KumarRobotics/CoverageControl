import torch
import matplotlib.pyplot as plt

class TrainModel():
    """
    Train a model using pytorch
    :param model: CNN model
    :param train_loader: training data loader
    :param test_loader: testing data loader
    :param optimizer: optimizer
    :param criterion: loss function
    :param epochs: number of epochs
    :param device: device
    :param model_file: model file
    :return: None
    """

    def __init__(self, model, train_loader, val_loader, test_loader, optimizer, criterion, epochs, device, model_file, optimizer_file):
        self.model = model
        self.train_loader = train_loader
        self.val_loader = val_loader
        self.test_loader = test_loader
        self.optimizer = optimizer
        self.criterion = criterion
        self.epochs = epochs
        self.device = device
        self.model_file = model_file
        self.optimizer_file = optimizer_file

    def LoadSavedModelDict(self, model_path):
        """
        Load the saved model
        :param model_path: model path
        :return: None
        """
        self.model.load_state_dict(torch.load(model_path))

    def LoadSavedModel(self, model_path):
        """
        Load the saved model
        :param model_path: model path
        :return: None
        """
        self.model = torch.load(model_path)

    def LoadSavedOptimizer(self, optimizer_path):
        """
        Load the saved optimizer
        :param optimizer_path: optimizer path
        :return: None
        """
        self.optimizer = torch.load(optimizer_path)

    # Train in batches, save the best model using the validation set
    def Train(self):
        """
        Train the model
        :return: None
        """
        # Initialize the best validation loss
        best_val_loss = float("Inf")

        # Initialize the loss history
        train_loss_history = []
        val_loss_history = []

        # Train the model
        for epoch in range(self.epochs):
            # Training
            train_loss = self.TrainEpoch()
            train_loss_history.append(train_loss)

            # Validation
            val_loss = self.ValidateEpoch()
            val_loss_history.append(val_loss)

            # Save the best model
            if val_loss < best_val_loss:
                best_val_loss = val_loss
                torch.save(self.model, self.model_file)
                torch.save(self.optimizer, self.optimizer_file)

            # Print the loss
            print("Epoch: {}/{}.. ".format(epoch + 1, self.epochs),
                  "Training Loss: {:.5f}.. ".format(train_loss),
                  "Validation Loss: {:.5f}.. ".format(val_loss))

            # Save the loss history
            model_path = self.model_file.split('.')[0]
            torch.save(train_loss_history, model_path + '_train_loss.pt')
            torch.save(val_loss_history, model_path + '_val_loss.pt')

    # Train the model in batches
    def TrainEpoch(self):
        """
        Train the model in batches
        :return: training loss
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

            # Clear the gradients
            self.optimizer.zero_grad()

            # Forward propagation
            output = self.model(data)

            if target.dim() == 3:
                target = target.view(-1, target.shape[-1])

            # Calculate the loss
            loss = self.criterion(output, target)

            # Print batch number and loss
            print("Batch: {}, Loss: {}".format(batch_idx, loss))

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
    def ValidateEpoch(self):
        """
        Validate the model in batches
        :return: validation loss
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

                # Forward propagation
                output = self.model(data)

                if target.dim() == 3:
                    target = target.view(-1, target.shape[-1])

                # Calculate the loss
                loss = self.criterion(output, target)

                # Update the validation loss
                val_loss += loss.item() * data.size(0)
                num_dataset += data.size(0)

        # Return the validation loss
        return val_loss / num_dataset

    # Test the model in batches
    def Test(self):
        """
        Test the model in batches
        :return: test loss
        """
        # Initialize the test loss
        test_loss = 0.0

        # Set the model to evaluation mode
        self.model.eval()

        num_dataset = 0
        # Test the model in batches
        with torch.no_grad():
            for batch_idx, (data, target) in enumerate(self.test_loader):
                # Move the data to the device
                data, target = data.to(self.device), target.to(self.device)

                # Forward propagation
                output = self.model(data)

                # Calculate the loss
                loss = self.criterion(output, target)

                if target.dim() == 3:
                    target = target.view(-1, target.shape[-1])

                # Update the test loss
                test_loss += loss.item() * data.size(0)
                num_dataset += data.size(0)

        # Return the test loss
        return test_loss / num_dataset
