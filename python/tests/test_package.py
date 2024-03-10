from __future__ import annotations

import importlib.metadata
import testcoverage as m


def test_version():
    assert importlib.metadata.version("testcoverage") == m.__version__
