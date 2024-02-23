# Start with cuda image
FROM nvidia/cuda:12.3.1-devel-ubuntu22.04

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

# Add repo for installing latest version of cmake
RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null; \
		echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ jammy main' | tee /etc/apt/sources.list.d/kitware.list >/dev/null; \
		apt-get update; \
		rm /usr/share/keyrings/kitware-archive-keyring.gpg

RUN apt install -y kitware-archive-keyring

RUN apt-get -y install cmake
RUN apt-get -y install \
											 libgmp-dev \
											 libmpfr-dev \
											 libboost-all-dev \
											 libeigen3-dev \
											 python3.10 \
											 python3.10-dev \
											 python3.10-venv \
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
RUN apt -y install ros-humble-desktop ros-dev-tools python3-colcon-common-extensions python3-vcstool python3-pip python3-argcomplete python3-rosdep python3-rosinstall-generator python3-rosinstall build-essential

# Remove cache to reduce image size
RUN rm -rf /var/lib/apt/lists/*; \
		rm -f /var/cache/apt/archives/*.deb; \
		rm -f /var/cache/apt/archives/parital/*.deb; \
		rm -f /var/cache/apt/*.bin

COPY requirements.txt /opt/requirements.txt
RUN python3.10 -m venv /opt/venv
RUN /opt/venv/bin/pip install --no-cache-dir wheel
RUN /opt/venv/bin/pip install --no-cache-dir -r /opt/requirements.txt

RUN mkdir -p /opt/dependencies/build; \
		mkdir -p /opt/dependencies/src
COPY install_dependencies.sh /opt/dependencies/install_dependencies.sh
RUN ["/bin/bash", "-c", "/opt/dependencies/install_dependencies.sh"]
RUN rm -r /opt/dependencies

ENV CoverageControl_ws /opt/CoverageControl_ws
ENV LD_LIBRARY_PATH ${CoverageControl_ws}/install/lib:/usr/local/lib:${LD_LIBRARY_PATH}
ENV PATH ${CoverageControl_ws}/install/bin:${PATH}
COPY .bashrc /root/.bashrc
RUN echo "source /opt/venv/bin/activate" >> /root/.bashrc

RUN mkdir -p ${CoverageControl_ws}
WORKDIR ${CoverageControl_ws}
