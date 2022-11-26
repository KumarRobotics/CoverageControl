# CoverageControl
Provides environment for Coverage Control problem.

## Requirements
    - CUDA nvcc
    - Eigen
    - Boost

## Installation

Copy the `setup.sh` file. It will clone the repository and install. 
Assuming main workspace directory: ${HOME}/CoverageControl_ws. Change `setup.sh` otherwise.

```
bash setup.sh
```
Add the following lines to your `.bashrc` file and then `source ~/.bashrc` so that Python can find the libraries:
```
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${HOME}/CoverageControl_ws/install/lib"
export PYTHONPATH="${PYTHONPATH}:${HOME}/CoverageControl_ws/install/lib"
```

Go to `CoverageControl/scripts/python` and check if the file `coverage.py` works. It contains minimal examples.
