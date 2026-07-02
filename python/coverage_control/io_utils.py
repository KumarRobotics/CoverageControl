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

from __future__ import annotations

import os
import sys
from pathlib import Path
from typing import Union, Dict, Any
import yaml

# Handle tomllib import based on Python version
if sys.version_info >= (3, 11):
    import tomllib
else:
    try:
        import tomli as tomllib
    except ImportError:
        tomllib = None


## @ingroup python_api
class IOUtils:
    """
    Class provides utility functions for loading data from various file formats.
    
    Methods:
        - load_tensor: Load PyTorch tensors from files
        - load_yaml: Load YAML configuration files
        - load_toml: Load TOML configuration files
        - sanitize_path: Sanitize and normalize file paths
    """

    @staticmethod
    def sanitize_path(path_str: str) -> str:
        """
        Sanitize and normalize a path string.
        
        Args:
            path_str (str): Path string to sanitize
            
        Returns:
            Path: Normalized pathlib.Path object
        """

        return os.path.normpath(os.path.expanduser(os.path.expandvars(path_str)))

    @staticmethod
    def load_tensor(path: Union[str, Path]) -> torch.Tensor:
        """
        Load a PyTorch tensor from a file.
        
        Supports loading tensors from both regular PyTorch saves and JIT script modules.
        
        Args:
            path (Union[str, Path]): Path to the tensor file
            
        Returns:
            torch.Tensor: The loaded tensor
            
        Raises:
            FileNotFoundError: If the file does not exist
            RuntimeError: If the file cannot be loaded or contains no tensor data
        """
        # torch is an optional dependency (the [nn] extra); import lazily so
        # the base package works without it
        try:
            import torch
        except ImportError as exc:
            raise ImportError(
                "IOUtils.load_tensor requires torch; "
                "install with: pip install coverage_control[nn]"
            ) from exc

        # Convert to Path object and sanitize
        if isinstance(path, str):
            path = IOUtils.sanitize_path(path)
        elif isinstance(path, Path):
            path = IOUtils.sanitize_path(str(path))
        else:
            raise TypeError(f"Path must be str or Path, got {type(path)}")
        path = Path(path)
        
        # Check if file exists
        if not path.is_file():
            raise FileNotFoundError(f"IOUtils::load_tensor: File not found: {path}")
        
        try:
            # Load data with weights_only for security
            data = torch.load(path, weights_only=True, map_location='cpu')
            
            # Extract tensor based on data type
            if isinstance(data, torch.jit.ScriptModule):
                # Get the first parameter from JIT script module
                params = list(data.parameters())
                if not params:
                    raise RuntimeError(f"No parameters found in JIT script module: {path}")
                tensor = params[0]
            elif isinstance(data, torch.Tensor):
                tensor = data
            elif isinstance(data, dict) and 'tensor' in data:
                # Handle case where tensor is stored in a dictionary
                tensor = data['tensor']
            else:
                raise RuntimeError(f"Unsupported data type in file: {type(data)}")
            
            return tensor
            
        except Exception as e:
            if isinstance(e, (FileNotFoundError, RuntimeError)):
                raise
            raise RuntimeError(f"Failed to load tensor from {path}: {str(e)}")
    
    @staticmethod
    def load_yaml(path: Union[str, Path]) -> Dict[str, Any]:
        """
        Load data from a YAML file.
        
        Args:
            path (Union[str, Path]): Path to the YAML file
            
        Returns:
            Dict[str, Any]: The loaded data as a dictionary
            
        Raises:
            FileNotFoundError: If the file does not exist
            yaml.YAMLError: If the YAML file is malformed
        """
        # Convert to Path object and sanitize
        if isinstance(path, str):
            path = IOUtils.sanitize_path(path)
        elif isinstance(path, Path):
            path = IOUtils.sanitize_path(str(path))
        else:
            raise TypeError(f"Path must be str or Path, got {type(path)}")
        
        path = Path(path)
        # Check if file exists
        if not path.is_file():
            raise FileNotFoundError(f"IOUtils::load_yaml: File not found: {path}")
        
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f)
            return data if data is not None else {}
            
        except yaml.YAMLError as e:
            raise yaml.YAMLError(f"Failed to parse YAML file {path}: {str(e)}")
        except Exception as e:
            raise RuntimeError(f"Failed to load YAML file {path}: {str(e)}")
    
    @staticmethod
    def load_toml(path: Union[str, Path]) -> Dict[str, Any]:
        """
        Load data from a TOML file.
        
        Args:
            path (Union[str, Path]): Path to the TOML file
            
        Returns:
            Dict[str, Any]: The loaded data as a dictionary
            
        Raises:
            FileNotFoundError: If the file does not exist
            ImportError: If tomllib/tomli is not available
            tomllib.TOMLDecodeError: If the TOML file is malformed
        """
        # Check if tomllib is available
        if tomllib is None:
            raise ImportError(
                "TOML support requires 'tomli' package for Python < 3.11. "
                "Install with: pip install tomli"
            )
        
        # Convert to Path object and sanitize
        if isinstance(path, str):
            path = IOUtils.sanitize_path(path)
        elif isinstance(path, Path):
            path = IOUtils.sanitize_path(str(path))
        else:
            raise TypeError(f"Path must be str or Path, got {type(path)}")
        
        path = Path(path)
        # Check if file exists
        if not path.is_file():
            raise FileNotFoundError(f"IOUtils::load_toml: File not found: {path}")
        
        try:
            with open(path, "rb") as f:
                data = tomllib.load(f)
            return data
            
        except tomllib.TOMLDecodeError as e:
            raise tomllib.TOMLDecodeError(f"Failed to parse TOML file {path}: {str(e)}")
        except Exception as e:
            raise RuntimeError(f"Failed to load TOML file {path}: {str(e)}")

