# CoverageControl
Provides environment for Coverage Control problem.

## Dependencies
```bash
- CUDA nvcc
- Eigen
- Boost
- yaml-cpp
```

Helper bash functions are provided in the file `setup.sh`.

## Installation

Copy the `setup.sh` file. It will clone the repository and install.   
Workspace directory: `${HOME}/CoverageControl_ws`.  See `setup.sh` file.  
It has installations for `pybind11` , `yaml-cpp`, and `eigen3` as well, which can be commented out if it has already been installed. See end of the file.

```bash
bash setup.sh
```
Add the following lines to your `.bashrc` file and then `source ~/.bashrc` so that Python can find the libraries:
```bash
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${HOME}/CoverageControl_ws/install/lib"
export PYTHONPATH="${PYTHONPATH}:${HOME}/CoverageControl_ws/install/lib"
```

Go to `CoverageControl/scripts/python` and check if the file `coverage.py` works. It contains minimal examples.
