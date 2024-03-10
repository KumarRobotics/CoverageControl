from __future__ import annotations

import importlib.metadata
import coverage_control as m


def test_version():
    assert importlib.metadata.version("coverage_control") == m.__version__
