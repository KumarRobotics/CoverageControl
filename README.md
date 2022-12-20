# CoverageControl

Provides environment for Coverage Control problem.

## Dependencies
System packages:
```bash
- CUDA nvcc
- Boost
- OpenMP
```

Python packages:
```bash
- pip
- numpy
- matplotlib
- pytz
```

## Installation

Workspace directory: `${HOME}/CoverageControl_ws`. Change as necessary.
```bash
export WORKSPACE_DIR=${HOME}/CoverageControl_ws
mkdir -p ${WORKSPACE_DIR}/src
git clone git@github.com:AgarwalSaurav/CoverageControl.git ${WORKSPACE_DIR}/src/CoverageControl
```

Install external packages:  
Inside `${WORKSPACE_DIR}/src/CoverageControl`
```bash
bash setup.sh -i
```

Add the following lines to your `.bashrc` file and then `source ~/.bashrc` or relogin:
```bash
export COVERAGECONTROL_PATH="${WORKSPACE_DIR}/install/"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${WORKSPACE_DIR}/install/lib/"
export PYTHONPATH="${PYTHONPATH}:${WORKSPACE_DIR}/install/lib/"
```

Install python API:
```bash
export WORKSPACE_DIR=${HOME}/CoverageControl_ws
pip install ${WORKSPACE_DIR}/src/CoverageControl
```

Go to `CoverageControl/scripts/python` and check if the file `coverage.py` works. It contains minimal examples.

## Update
Inside `${WORKSPACE_DIR}/src/CoverageControl`

```bash
bash setup.sh -u
pip install .
```
