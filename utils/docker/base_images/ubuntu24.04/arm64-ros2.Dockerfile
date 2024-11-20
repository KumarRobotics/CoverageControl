FROM arm64v8/ubuntu:24.04

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

ARG PYTHON_VERSION="3.12"
ARG PYTORCH_VERSION="2.5.1"

ENV PYTHON_VERSION=${PYTHON_VERSION}
ENV PYTORCH_VERSION=${PYTORCH_VERSION}

ENV TERM=xterm-256color

RUN apt-get update && apt-get install -y apt-utils

RUN apt-get	-y update; \
		apt-get -y upgrade; \
		apt-get -y install \
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
											 python-is-python3

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
											 ninja-build libpng-dev libjpeg-dev libopencv-dev python3-opencv

RUN add-apt-repository universe
RUN locale-gen en_US en_US.UTF-8; update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8; export LANG=en_US.UTF-8
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get -y update && apt-get -y upgrade
RUN apt-get -y install ros-jazzy-desktop ros-dev-tools python3-colcon-common-extensions python3-vcstool python3-pip python3-argcomplete python3-rosdep python3-rosinstall-generator build-essential

# Remove cache to reduce image size
RUN rm -rf /var/lib/apt/lists/*; \
		rm -f /var/cache/apt/archives/*.deb; \
		rm -f /var/cache/apt/archives/parital/*.deb; \
		rm -f /var/cache/apt/*.bin

RUN mkdir -p /opt
RUN mkdir download; \
		wget https://github.com/AgarwalSaurav/libtorch_arm64/releases/download/v${PYTORCH_VERSION}/libtorch-cxx11-abi-shared-with-deps-${PYTORCH_VERSION}.zip -O download/libtorch.zip; \
		unzip download/libtorch.zip -d /opt/; \
		rm -r download

ENV LD_LIBRARY_PATH="/usr/local/lib:/opt/libtorch/lib"
ENV Torch_DIR=/opt/libtorch/share/cmake/

COPY requirements_cpu.txt /opt/requirements.txt
RUN python${PYTHON_VERSION} -m venv /opt/venv
RUN /opt/venv/bin/pip install --no-cache-dir wheel setuptools==68.1.2
RUN /opt/venv/bin/pip install --no-cache-dir -r /opt/requirements.txt
RUN /opt/venv/bin/pip install --no-cache-dir catkin_pkg lark
ENV VENV_PATH=/opt/venv

COPY .ros.jazzy.bashrc /root/.bashrc
