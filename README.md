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
Add the following lines to your `.bashrc` file and then `source ~/.bashrc` or relogin:
```bash
export COVERAGECONTROL_WS="${HOME}/CoverageControl_ws/"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${COVERAGECONTROL_WS}/install/lib/"
export PYTHONPATH="${PYTHONPATH}:${COVERAGECONTROL_WS}/install/lib/"
```

```bash
mkdir -p ${COVERAGECONTROL_WS}/src
git clone git@github.com:AgarwalSaurav/CoverageControl.git ${COVERAGECONTROL_WS}/src/CoverageControl
```

Install external packages:  
Inside `${COVERAGECONTROL_WS}/src/CoverageControl`
```bash
bash setup.sh -i
```


Install python API:
```bash
pip install ${COVERAGECONTROL_WS}/src/CoverageControl
```

Go to `CoverageControl/scripts/python` and check if the file `coverage.py` works. It contains minimal examples.

## Update
Inside `${COVERAGECONTROL_WS}/src/CoverageControl`

```bash
bash setup.sh -u
pip install .
```
