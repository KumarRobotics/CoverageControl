# This file is part of the CoverageControl library
#
# Author: Saurav Agarwal
# Contact: sauravag@seas.upenn.edu, agr.saurav1@gmail.com
# Repository: https://github.com/KumarRobotics/CoverageControl
#
# Copyright (c) 2024, Saurav Agarwal
#
# The CoverageControl library is free software: you can redistribute it and/or
# modify it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or (at your
# option) any later version.
#
# The CoverageControl library is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
# Public License for more details.
#
# You should have received a copy of the GNU General Public License along with
# CoverageControl library. If not, see <https://www.gnu.org/licenses/>.

## @file io_utils.py
#  @brief The module provides utility functions for loading data from files

"""
The module provides utility functions for loading data from files
"""

import os
import sys
import pathlib

import torch
import yaml

if sys.version_info[1] < 11:
    import tomli as tomllib
else:
    import tomllib

## @ingroup python_api
class IOUtils:
    """
    Class provides the following utility functions:
    - load_tensor
    - load_yaml
    - load_toml
    """

    @staticmethod
    def sanitize_path(path_str: str) -> str:
        """
        Function to sanitize a path string
        """

        return os.path.normpath(os.path.expanduser(os.path.expandvars(path_str)))

    @staticmethod
    def load_tensor(path: (str, pathlib.Path)) -> torch.tensor:
        """
        Function to load a tensor from a file
        Can load tensors from jit script format files

        Args:
            path (str): Path to the file

        Returns:
            tensor: The loaded tensor
            None: If the file does not exist

        Raises:
            FileNotFoundError: If the file does not exist
        """

        if isinstance(path, pathlib.Path):
            path = str(path)
        # Throw error if path does not exist
        path = IOUtils.sanitize_path(path)

        if not os.path.exists(path):
            raise FileNotFoundError(f"IOUtils::load_tensor: File not found: {path}")
        # Load data
        data = torch.load(path, weights_only=True)
        # Extract tensor if data is in jit script format

        if isinstance(data, torch.jit.ScriptModule):
            tensor = list(data.parameters())[0]
        else:
            tensor = data

        return tensor

    @staticmethod
    def load_yaml(path: str) -> dict:
        """
        Function to load a yaml file

        Args:
            path (str): Path to the file

        Returns:
            data: The loaded data

        Raises:
            FileNotFoundError: If the file does not exist
        """

        path = IOUtils.sanitize_path(path)
        # Throw error if path does not exist

        if not os.path.exists(path):
            raise FileNotFoundError(f"IOUtils::load_yaml File not found: {path}")
        # Load data
        with open(path, "rb") as f:
            data = yaml.load(f, Loader=yaml.FullLoader)

        return data

    @staticmethod
    def load_toml(path: str) -> dict:  # Throw error if path does not exist
        """
        Function to load a toml file
        """
        path = IOUtils.sanitize_path(path)

        if not os.path.exists(path):
            raise FileNotFoundError(f"IOUtils::load_toml: File not found: {path}")
        # Load data
        with open(path, "rb") as f:
            data = tomllib.load(f)

        return data
