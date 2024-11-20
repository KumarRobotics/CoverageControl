\page installation Installation
\tableofcontents

# PyPI Installation
The library is available on PyPI and can be installed using `pip`.
It is recommended to install the library inside a virtual environment.
```bash
pip install coverage_control
```

The package depends on the following packages
- [PyTorch](https://pytorch.org/)
- [PyTorch Geometric](https://pytorch-geometric.readthedocs.io/en/latest/)

```bash
pip install torch torchvision torch-geometric
```

\note PyTorch and PyTorch Geometric have CPU and CUDA-specific versions. The command installs the default version (latest CUDA).

We need the following optional packages for visualization and video generation:
- `gnuplot` or `gnuplot-nox` (for visualizing environment)
- `ffmpeg` (for generating videos)

On Ubuntu, these can be installed using the following command:
```bash
sudo apt install gnuplot-nox ffmpeg
```

--------

# Docker Installation

Docker images are available for the library with different configurations and versions.

## Prerequisites (Optional)
We will organize files in a **workspace** directory: `${CoverageControl_ws}` (e.g., ~/CoverageControl\_ws).
The workspace directory is mounted to the docker container.

Add the following lines to your `${HOME}/.bashrc` file for convenience.
```bash
export CoverageControl_ws=${HOME}/CoverageControl_ws # Change to your workspace directory
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
Container can be created using the script in `utils/docker/create_container.sh`.
```bash
cd ${CoverageControl_ws}/src/CoverageControl/utils/docker
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
- `--noble` : Ubuntu 24.04 Noble

The base image is `agarwalsaurav/pytorch_base` with different tags for different versions and configurations.

|Tags Suffix | Flags|
|--- | ---|
|`jammy-torch2.5.1-cuda12.4.1-humble` | `--with-ros --with-cuda`|
|`jammy-torch2.5.1-cuda12.4.1` | `--with-cuda`|
|`jammy-torch2.5.1-humble` | `--with-ros`|
|`jammy-torch2.5.1` | None|
|`noble-torch2.5.1-cuda12.6.2-jazzy` | `--with-ros --with-cuda --noble`|
|`noble-torch2.5.1-cuda12.6.2` | `--with-cuda --noble`|
|`noble-torch2.5.1-jazzy` | `--with-ros --noble`|
|`noble-torch2.5.1` | `--noble`|


Install the library available on PyPI:
```bash
pip install coverage_control
```

Alternatively, follow the [Installation from Source](#installation-from-source) instructions (except for the prerequisites).

--------

# Installation From Source {#installation-from-source}
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

## Automated Installation

```bash
pip install .
```

## Building the Core C++ Library


\note This is only necessary if you want to use the C++ library directly.

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

Testing the installation:
```bash
coverage_algorithm
```

There are multiple options for building the library.

Option | Description
--- | ---
`-d <dir>` | The workspace directory
`--with-cuda` | Build with CUDA support


\warning Ubuntu 22.04 (Jammy) has CGAL 5.4 (libcgal-dev) in the official repositories, which has bugs and is not compatible with the library. The package requires `CGAL 5.6`, which is automatically installed from the official CGAL repository through `CMake`.

--------
