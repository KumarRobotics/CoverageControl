"""
Provides CVT-based algorithms for coverage control.
"""

from __future__ import annotations

from .._core import CentralizedCVT, ClairvoyantCVT, DecentralizedCVT, NearOptimalCVT

__all__ = ["NearOptimalCVT", "ClairvoyantCVT", "CentralizedCVT", "DecentralizedCVT"]
