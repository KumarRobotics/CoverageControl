\page docker Docker
\tableofcontents

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

The base image is `agarwalsaurav/coveragecontrol` with different tags for different versions and configurations.

|Tags Suffix | Flags|
|--- | ---|
|`python2.2.2-cuda12.2.2-ros2humble` | `--with-ros --with-cuda`|
|`python2.2.2-cuda12.2.2` | `--with-cuda`|
|`python2.2.2-ros2humble` | `--with-ros`|
|`python2.2.2` | None|

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
