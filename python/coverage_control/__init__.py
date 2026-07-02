"""
Copyright (c) 2024 Saurav Agarwal. All rights reserved.

coverage_control: Library for large-scale coverage control using robot swarms
"""

from __future__ import annotations

import os.path as _osp

from ._version import version as __version__
from .core import *
from .io_utils import IOUtils

try:
    from .coverage_env_utils import CoverageEnvUtils
except ImportError:
    # torch/torch_geometric/torchvision are optional (the [nn] extra);
    # CoverageEnvUtils is unavailable without them
    CoverageEnvUtils = None

# from .nn import *

__all__ = ["__version__", "core", "nn", "IOUtils", "CoverageEnvUtils"]

cmake_prefix_path = _osp.join(_osp.dirname(_osp.dirname(__file__)), "lib", "cmake")
