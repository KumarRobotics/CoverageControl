# CoverageControl
Provides environment for Coverage Control problem.

## Requirements
    - CUDA nvcc
    - Eigen
    - Boost
    - yaml-cpp

## Installation

Copy the `setup.sh` file. It will clone the repository and install.   
Assuming main workspace directory: `${HOME}/CoverageControl_ws`. Change inside `setup.sh` otherwise.

```bash
bash setup.sh
```
Add the following lines to your `.bashrc` file and then `source ~/.bashrc` so that Python can find the libraries:
```bash
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${HOME}/CoverageControl_ws/install/lib"
export PYTHONPATH="${PYTHONPATH}:${HOME}/CoverageControl_ws/install/lib"
```

Go to `CoverageControl/scripts/python` and check if the file `coverage.py` works. It contains minimal examples.

If you make changes to the CoverageControl repository, run the last two lines in `setup.sh` file by appropriately substituting `${BUILD_DIR}`
