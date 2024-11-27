ARG IMAGE_TYPE="cpu"
ARG CUDA_VERSION="12.4.1"
ARG UBUNTU_VERSION="22.04"

FROM nvidia/cuda:${CUDA_VERSION}-devel-ubuntu${UBUNTU_VERSION} AS cuda
ARG PYTORCH_VERSION="2.5.1"
ENV PYTORCH_VERSION=${PYTORCH_VERSION}
COPY requirements.txt /opt/requirements.txt
RUN mkdir download; \
    wget https://download.pytorch.org/libtorch/cu124/libtorch-win-shared-with-deps-${PYTORCH_VERSION}%2Bcu124.zip -O download/libtorch.zip; \
		unzip download/libtorch.zip -d /opt/; \
		rm -r download

FROM ubuntu:${UBUNTU_VERSION} AS cpu
ARG PYTORCH_VERSION="2.5.1"
ENV PYTORCH_VERSION=${PYTORCH_VERSION}
COPY requirements_cpu.txt /opt/requirements.txt
RUN mkdir download; \
		wget https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-${PYTORCH_VERSION}%2Bcpu.zip -O download/libtorch.zip; \
		unzip download/libtorch.zip -d /opt/; \
		rm -r download

FROM arm64v8/ubuntu:${UBUNTU_VERSION} AS arm64
ARG PYTORCH_VERSION="2.5.1"
ENV PYTORCH_VERSION=${PYTORCH_VERSION}
COPY requirements_cpu.txt /opt/requirements.txt
RUN mkdir download; \
		wget https://github.com/AgarwalSaurav/libtorch_arm64/releases/download/v${PYTORCH_VERSION}/libtorch-cxx11-abi-shared-with-deps-${PYTORCH_VERSION}.zip -O download/libtorch.zip; \
		unzip download/libtorch.zip -d /opt/; \
		rm -r download

FROM ${IMAGE_TYPE} AS base
ARG PYTHON_VERSION="3.10"
LABEL maintainer="Saurav Agarwal <sauravag@upenn.edu>"
LABEL description="Dockerfile for PyTorch GPU and ROS2"

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHON_VERSION=${PYTHON_VERSION}
ENV TERM=xterm-256color

RUN apt-get update && apt-get install -y apt-utils

RUN apt-get	-y update \
		&& apt-get -y upgrade \
		&& apt-get -y install \
											 build-essential \
											 git \
											 wget \
											 gpg \
											 curl \
											 gdb \
                       libbz2-dev \
											 software-properties-common \
											 ca-certificates \
											 lsb-release \
											 net-tools iputils-ping \
											 locales \
                       python${PYTHON_VERSION} \
                       python${PYTHON_VERSION}-dev \
                       python${PYTHON_VERSION}-venv \
    && apt-get -y autoremove \
    && apt-get -y clean autoclean

# Add repo for installing latest version of cmake
RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null; \
		echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ jammy main' | tee /etc/apt/sources.list.d/kitware.list >/dev/null; \
		apt-get -y update; \
		rm /usr/share/keyrings/kitware-archive-keyring.gpg \
    && apt-get -y install kitware-archive-keyring

RUN apt-get -y install \
											 cmake \
											 libgmp-dev \
											 libmpfr-dev \
											 libboost-all-dev \
											 libeigen3-dev \
											 libgeos-dev \
											 libyaml-cpp-dev \
											 vim \
                       neovim \
											 tmux \
											 ffmpeg \
											 unzip \
											 gnuplot-nox \
											 ninja-build \
                       libpng-dev \
                       libjpeg-dev \
                       libopencv-dev \
                       python3-opencv \
    && apt-get -y autoremove \
    && apt-get -y clean autoclean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* \
    && rm -f /var/cache/apt/archives/*.deb \
    && rm -f /var/cache/apt/archives/parital/*.deb \
    && rm -f /var/cache/apt/*.bin

ENV LD_LIBRARY_PATH="/usr/local/lib:/opt/libtorch/lib"
ENV Torch_DIR=/opt/libtorch/share/cmake/

RUN python${PYTHON_VERSION} -m venv /opt/venv \
    && /opt/venv/bin/pip install --no-cache-dir wheel \
    && /opt/venv/bin/pip install --no-cache-dir -r /opt/requirements.txt
ENV VENV_PATH=/opt/venv

COPY .bashrc /root/.bashrc

FROM base AS ros2
ARG ROS_DISTRO="humble"
ENV ROS_DISTRO=${ROS_DISTRO}
RUN apt-get -y update \
    && add-apt-repository universe \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && export LANG=en_US.UTF-8 \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get -y update \
    && apt-get install -y \
                       python3-colcon-common-extensions \
                       python3-vcstool \
                       python3-pip \
                       python3-argcomplete \
                       python3-rosdep \
                       python3-rosinstall-generator \
                       ros-${ROS_DISTRO}-desktop \
                       ros-dev-tools \
                       ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
                       ros-${ROS_DISTRO}-ros-gz \
                       ros-${ROS_DISTRO}-ros2bag \
                       ros-${ROS_DISTRO}-rosidl-generator-dds-idl \
    && apt-get -y autoremove \
    && apt-get -y clean autoclean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* \
    && rm -f /var/cache/apt/archives/*.deb \
    && rm -f /var/cache/apt/archives/parital/*.deb \
    && rm -f /var/cache/apt/*.bin

RUN /opt/venv/bin/pip install --no-cache-dir catkin_pkg empy==3.3.4 lark tomli
RUN echo $'\n\
export PYTHONPATH=/opt/venv/lib/python${PYTHON_VERSION}/site-packages:$PYTHONPATH\n\
export ROS_PYTHON_VERSION=3\n\
export ROS_VERSION=2\n\
export ROS_DOMAIN_ID=10\n\
source /opt/ros/${ROS_DISTRO}/setup.bash\n\
source /usr/share/colcon_cd/function/colcon_cd.sh\n\
export _colcon_cd_root=/opt/ros/${ROS_DISTRO}' >> /root/.bashrc
