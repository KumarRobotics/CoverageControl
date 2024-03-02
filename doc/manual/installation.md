\page installation Installation
\tableofcontents

This page provides instructions for installing the Coverage Control library.
The library can be installed using Docker or from source code.
The Docker installation is the easiest way to get started, but the source installation provides more flexibility and control over the installation and usage of the library.

# Docker Installation

## Prerequisites (Optional)
We will organize files in a **workspace** directory: `${CoverageControl_ws}` (e.g., ~/CoverageControl\_ws).
The workspace directory is mounted to the docker container.

Add the following lines to your `~/.bashrc` file for convenience.
```bash
export CoverageControl_ws=~/CoverageControl_ws # Change to your workspace directory
```
\note Close and reopen the terminal for the changes to take effect.

Clone the repository:
```bash
mkdir -p ${CoverageControl_ws}/src
git clone https://github.com/KumarRobotics/CoverageControl.git \
          ${CoverageControl_ws}/src/CoverageControl
```

--------

## Docker Container
Container can be created using the script in `setup_utils/create_container.sh`.
```bash
cd ${CoverageControl_ws}/src/CoverageControl/setup_utils
bash create_container.sh --with-cuda -d ${CoverageControl_ws} # See flags below
```

This will land you in a shell inside the container in the directory `/workspace`.
If the workspace directory was specified, it will be mounted to the container at the same location.

One can exit the container by typing `exit` or pressing `Ctrl+D`.

The container can be started again using the following command:
```bash
docker start -i coverage-control-$USER # Replace with the name of the container
```


**Flags:**
- `-d <dir>` : The workspace directory
- `-n <name>`: Name of the container (default: `coverage-control-$USER`)
- `--with-cuda` : With CUDA support
- `--with-ros` : With ROS support

The base image is `ghcr.io/\repo_owner_lower/coveragecontrol` with different tags for different versions and configurations.

|Tags Suffix | Flags|
|--- | ---|
|`python2.2.1-cuda12.3.1-ros2humble` | `--with-ros --with-cuda`|
|`python2.2.1-cuda12.3.1` | `--with-cuda`|
|`python2.2.1-ros2humble` | `--with-ros`|
|`python2.2.1` | None|

--------

## Building and Executing

The library is already built and installed in the container.
However, if you want to build it again, you can do so using the following commands.

The primary setup script is `setup.sh` located in the root of the repository.
```bash
cd ${CoverageControl_ws}/src/CoverageControl
bash setup.sh -p --with-cuda -d ${CoverageControl_ws}
```

There are multiple options for building the library.

Option | Description
--- | ---
`-d [dir]` | The workspace directory
`-p` | Build and install `python` bindings and `CoverageControlTorch` package
`-g` | Installs globally (builds inside the workspace directory if `-d` is specified)
`--with-cuda` | Build with CUDA support
`--with-deps` | Install dependencies (Not required if using the docker image)

**Testing:**
```bash
coverage_algorithm
```

# Installation from source
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
