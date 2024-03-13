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

'''
Utility functions for combining and splitting datasets.
'''

import os
import sys
import yaml
import torch
if sys.version_info[1] < 11:
    import tomli as tomllib
else:
    import tomllib

def load_tensor(path):
    """
    Function to load a tensor from a file
    Checks if the file exists, if not, returns None
    Checks if the loaded data is a tensor or is in jit script format
    """
    # Throw error if path does not exist
    if not os.path.exists(path):
        raise FileNotFoundError(f"data_loader_utils::load_tensor: File not found: {path}")
    # Load data
    data = torch.load(path)
    # Extract tensor if data is in jit script format
    if isinstance(data, torch.jit.ScriptModule):
        tensor = list(data.parameters())[0]
    else:
        tensor = data
    return tensor

def load_toml(path): # Throw error if path does not exist
    if not os.path.exists(path):
        raise FileNotFoundError(f"data_loader_utils::LoadToml: File not found: {path}")
    # Load data
    with open(path, "rb") as f:
        data = tomllib.load(f)
    return data

def load_yaml(path):
    # Throw error if path does not exist
    if not os.path.exists(path):
        raise FileNotFoundError(f"data_loader_utils::load_yaml: File not found: {path}")
    # Load data
    with open(path, "r") as f:
        data = yaml.load(f, Loader=yaml.FullLoader)
    return data

def split_save_data(path, data_name, num_train, num_val, num_test):
    data = load_tensor(path + data_name + '.pt')
    # Check if tensor is sparse, if so, convert to dense
    is_sparse = False
    if data.is_sparse:
        data = data.to_dense()
        is_sparse = True
    # Split data into training, validation, and test sets
    train_data = data[:num_train].clone()
    val_data = data[num_train:num_train+num_val].clone()
    test_data = data[num_train+num_val:].clone()
    # Save data
    if is_sparse:
        train_data = train_data.to_sparse()
        val_data = val_data.to_sparse()
        test_data = test_data.to_sparse()

    torch.save(train_data, path + '/train/' + data_name + '.pt')
    torch.save(val_data, path + '/val/' + data_name + '.pt')
    torch.save(test_data, path + '/test/' + data_name + '.pt')
    # delete data to free up memory
    del data
    del train_data
    del val_data
    del test_data
    # delete data file
    os.remove(path + data_name + '.pt')

def normalize_data(data):
    data_mean = torch.mean(data.view(-1, data.shape[-1]), dim=0)
    data_std = torch.std(data.view(-1, data.shape[-1]), dim=0)
    data = (data - data_mean) / data_std
    return data_mean, data_std, data

def split_dataset(config_path):
    '''
    Split dataset into training, validation, and test sets.
    The information is received via yaml config file.
    '''
    config = load_toml(os.path.expanduser(config_path))
    data_path = config['DataDir']
    data_path = os.path.expanduser(data_path)
    data_dir = data_path + '/data/'

    train_dir = data_dir + '/train'
    val_dir = data_dir + '/val'
    test_dir = data_dir + '/test'

    # Create directories if they don't exist
    if not os.path.exists(train_dir):
        os.makedirs(train_dir)
    if not os.path.exists(val_dir):
        os.makedirs(val_dir)
    if not os.path.exists(test_dir):
        os.makedirs(test_dir)

    num_dataset = config['NumDataset']
    train_ratio = config['DataSetSplit']['TrainRatio']
    val_ratio = config['DataSetSplit']['ValRatio']

    num_train = int(train_ratio * num_dataset)
    num_val = int(val_ratio * num_dataset)
    num_test = num_dataset - num_train - num_val

    data_names = ['local_maps', 'comm_maps', 'obstacle_maps', 'actions', 'normalized_actions', 'edge_weights', 'robot_positions', 'coverage_features', 'normalized_coverage_features']
    for data_name in data_names:
        split_save_data(data_dir, data_name, num_train, num_val, num_test)

def combine_dataset(config_path, subdir_list):
    '''
    Combine split datasets into one dataset.
    The information is received via yaml config file.
    subdir_list is a list of subdirectories to combine, e.g. ['0', '1', '2']
    '''
    config = load_toml(os.path.expanduser(config_path))
    data_path = config['DataDir']
    data_path = os.path.expanduser(data_path)
    data_dir = data_path + '/data/'
    # Create directory if it doesn't exist

    data_names = ['local_maps', 'comm_maps', 'obstacle_maps', 'actions', 'edge_weights', 'robot_positions', 'coverage_features']
    normalize_data_names = ['actions', 'coverage_features']
    for data_name in data_names:
        is_sparse = False
        for subdir in subdir_list:
            data = load_tensor(data_dir + subdir + '/' + data_name + '.pt')
            if subdir == subdir_list[0]:
                is_sparse = data.is_sparse
                if is_sparse:
                    data = data.to_dense()
                combined_data = data.clone()
            else:
                if is_sparse:
                    data = data.to_dense()
                combined_data = torch.cat((combined_data, data.clone()), dim=0)
            del data
        if is_sparse:
            combined_data = combined_data.to_sparse()
        torch.save(combined_data, data_dir + data_name + '.pt')
        if data_name in normalize_data_names:
            combined_data = combined_data.to_dense()
            combined_data_mean, combined_data_std, combined_data = normalize_data(combined_data)
            if is_sparse:
                combined_data = combined_data.to_sparse()
            torch.save(combined_data_mean, data_dir + data_name + '_mean.pt')
            torch.save(combined_data_std, data_dir + data_name + '_std.pt')
            torch.save(combined_data, data_dir + 'normalized_' + data_name + '.pt')
        del combined_data

__all__ = ['split_dataset', 'combine_dataset']

# Example of how to call split_dataset from terminal
# python -c 'from dataset_utils import split_dataset; split_dataset("config.yaml")'

# Example of how to call combine_dataset from terminal
# python -c 'from dataset_utils import combine_dataset; combine_dataset("config.yaml", ["0", "1", "2"])'

