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
```

May need to add the following as well if not using `pip` to install `python` package:
```bash
export PYTHONPATH="${PYTHONPATH}:${COVERAGECONTROL_WS}/install/lib/"
```

Clone the repository:
```bash
mkdir -p ${COVERAGECONTROL_WS}/src
git clone git@github.com:AgarwalSaurav/CoverageControl.git ${COVERAGECONTROL_WS}/src/CoverageControl
```

Install `C++` packages:  
```bash
cd ${COVERAGECONTROL_WS}/src/CoverageControl/
bash setup.sh -i
```


Install python API (can be done within `conda` environment):
```bash
cd ${COVERAGECONTROL_WS}/src/CoverageControl
pip install .
```

Go to `CoverageControl/core/scripts/python` and check if the file `coverage.py` works. It contains minimal examples.

## Update
When the backend C++ code updates:
```bash
cd ${COVERAGECONTROL_WS}/src/CoverageControl
git pull
bash setup.sh -u
pip install .
```


## Uninstall
```bash
pip uninstall pyCoverageControl
rm -r ${COVERAGECONTROL_WS}/
```
