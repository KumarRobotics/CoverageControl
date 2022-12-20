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

Workspace directory: `${HOME}/CoverageControl_ws`. Assumes that the directory does not exist.  
Add the following lines to your `.bashrc` file and then `source ~/.bashrc` or open a new terminal:
```bash
export COVERAGECONTROL_WS="${HOME}/CoverageControl_ws/"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${COVERAGECONTROL_WS}/install/lib/"
export PYTHONPATH="${PYTHONPATH}:${COVERAGECONTROL_WS}/install/lib/"
```

Clone the repository:
```bash
mkdir -p ${COVERAGECONTROL_WS}/src
git clone git@github.com:AgarwalSaurav/CoverageControl.git ${COVERAGECONTROL_WS}/src/CoverageControl
```

Install external packages:  
```bash
cd ${COVERAGECONTROL_WS}/src/CoverageControl/core
bash setup.sh -i
```


Install python API (can be done within `conda` environment):
```bash
cd ${COVERAGECONTROL_WS}/src/CoverageControl/core
pip install .
```

Go to `CoverageControl/core/scripts/python` and check if the file `coverage.py` works. It contains minimal examples.

## Update
When the backend C++ code updates:
```bash
cd ${COVERAGECONTROL_WS}/src/CoverageControl/core
git pull
bash setup.sh -u
pip install .
```
