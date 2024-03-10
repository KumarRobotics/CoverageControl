import torch
import matplotlib.pyplot as plt

class MultiTrainModel():
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

    def __init__(self, models, train_loader, val_loader, test_loader, optimizers, criterion, epochs, devices, model_files, optimizer_files):
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
    def TrainEpoch(self):
        """
        Train the model in batches
        :return: training loss
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
    def ValidateEpoch(self):
        """
        Validate the model in batches
        :return: validation loss
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
    def Test(self):
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
