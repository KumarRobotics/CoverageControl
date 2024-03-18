pip install --no-build-isolation --config-settings=editable.rebuild=true -Cbuild-dir=build -ve.
pytest --ignore=python/tests/deprecated python/tests/ --ignore=python/tests/test_lpac_coverage.py -vv
