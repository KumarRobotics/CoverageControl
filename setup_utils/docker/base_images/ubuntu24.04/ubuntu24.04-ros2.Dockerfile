FROM ubuntu:24.04

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND noninteractive

# Post actions after apt installs cause errors. This has been fixed in more recent versions of docker
RUN sed -i -e 's/^APT/# APT/' -e 's/^DPkg/# DPkg/' \
      /etc/apt/apt.conf.d/docker-clean

ENV TERM xterm-256color

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
											 software-properties-common \
											 ca-certificates \
											 lsb-release \
											 net-tools iputils-ping \
											 locales

RUN apt-get -y install \
											 cmake \
											 libgmp-dev \
											 libmpfr-dev \
											 libboost-all-dev \
											 libeigen3-dev \
											 python3.12 \
											 python3.12-dev \
											 python3.12-venv \
											 python-is-python3 \
											 libgeos-dev \
											 libyaml-cpp-dev \
											 vim \
											 tmux \
											 ffmpeg \
											 gnuplot-nox \
											 ninja-build libpng-dev libjpeg-dev libopencv-dev python3-opencv

RUN add-apt-repository universe
RUN locale-gen en_US en_US.UTF-8; update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8; export LANG=en_US.UTF-8
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get -y update && apt-get -y upgrade
RUN apt -y install ros-jazzy-desktop ros-dev-tools python3-colcon-common-extensions python3-vcstool python3-pip python3-argcomplete python3-rosdep python3-rosinstall-generator python3-colcon-argcomplete

# Remove cache to reduce image size
RUN rm -rf /var/lib/apt/lists/*; \
		rm -f /var/cache/apt/archives/*.deb; \
		rm -f /var/cache/apt/archives/parital/*.deb; \
		rm -f /var/cache/apt/*.bin

RUN mkdir download; \
		wget https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.3.1%2Bcpu.zip -O download/libtorch.zip; \
		unzip download/libtorch.zip -d /opt/; \
		rm -r download

ENV LD_LIBRARY_PATH /usr/local/lib:/opt/libtorch/lib:${LD_LIBRARY_PATH}
ENV Torch_DIR /opt/libtorch/share/cmake/

COPY requirements_cpu.txt /opt/requirements.txt
RUN python3.12 -m venv /opt/venv
RUN /opt/venv/bin/pip install --no-cache-dir wheel
RUN /opt/venv/bin/pip install --no-cache-dir -r /opt/requirements.txt
ENV VENV_PATH /opt/venv

ENV LD_LIBRARY_PATH /usr/local/lib:${LD_LIBRARY_PATH}
COPY .ros.bashrc /root/.bashrc
