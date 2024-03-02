\page installation-from-source Installation From Source
\tableofcontents

## Prerequisites

The following packages are required to build the library:
```bash
sudo apt install libboost-all-dev libgmp-dev libmpfr-dev libeigen3-dev gnuplot-nox ffmpeg
```
\note `gnuplot-nox` and `ffmpeg` are optional (but recommended) and only required for generating environment visualizations.

Additional dependencies (generally already installed):
```bash
sudo apt install build-essential cmake git wget python3 python3-pip python3-venv python3-dev
```

### CUDA Support
(Optional but recommended for GPU acceleration)

The package also supports GPU acceleration using CUDA. To enable this feature, the following additional packages are required:
- [`cmake`](https://cmake.org/download/) (version 3.24 or higher)
- `cuda` (version 11.8 or higher, 12.1 recommended)

\note On Ubuntu, latest `cmake` version can be installed from the official [Kitware APT Repository](https://apt.kitware.com/).

--------

## Building the Core Library

We will organize files in a **workspace** directory: `${CoverageControl_ws}` (e.g., ~/CoverageControl\_ws).

Add the following lines to your `~/.bashrc` file.
```bash
export CoverageControl_ws=~/CoverageControl_ws # Change to your workspace directory
export PATH=${CoverageControl_ws}/install/bin:$PATH
export LD_LIBRARY_PATH=${CoverageControl_ws}/install/lib:$LD_LIBRARY_PATH
```

Clone the repository:
```bash
mkdir -p ${CoverageControl_ws}/src
git clone https://github.com/KumarRobotics/CoverageControl.git \
          ${CoverageControl_ws}/src/CoverageControl
```

The primary setup script is `setup.sh` located in the root of the repository.
```bash
cd ${CoverageControl_ws}/src/CoverageControl
bash setup.sh --with-deps -d ${CoverageControl_ws}
```

There are multiple options for building the library.

Option | Description
--- | ---
`-d <dir>` | The workspace directory
`-p` | Build and install `python` packages (See [CoverageControlTorch Python Package](#coveragecontroltorch-python-package))
`--with-cuda` | Build with CUDA support
`--with-deps` | Install dependencies (CGAL 5.6)


\warning Ubuntu 22.04 (Jammy) has CGAL 5.4 (libcgal-dev) in the official repositories, which has bugs and is not compatible with the library. The package requires `CGAL 5.6`, which is installed if `--with-deps` is used. The `--with-deps` option is only required for the first build as the downloaded files will persist in the workspace installation directory (`${CoverageControl_ws}/install`).

--------

## Python Packages

The library provides two `python` packages:
- `%CoverageControl` (bindings for the core library)
- `%CoverageControlTorch` (classes, utilities, and scripts for training and evaluating neural network policies)

These can be installed by adding the `-p` option to the `setup.sh` script:
```bash
cd ${CoverageControl_ws}/src/CoverageControl
bash setup.sh -p -d ${CoverageControl_ws}
```
\note It is recommended that `python` bindings are built inside a virtual environment.

Test the installation by running the following commands:
```bash
python ${CoverageControl_ws}/src/CoverageControl/python/tests/coverage_algorithm.py
```

The `CoverageControlTorch` is built on top of `pytorch` and `pytorch-geometric`. Depending of whether you have `CUDA` installed, you can use either of the following files to install the package:
- `setup_utils/requirements.txt` (for GPU)
- `setup_utils/requirements_cpu.txt` (for CPU)

\note Please change the `torch` and `torch-geometric` versions in the file to match your CUDA version.
