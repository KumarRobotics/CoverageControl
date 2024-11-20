FROM ubuntu:24.04

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

# Remove cache to reduce image size
RUN rm -rf /var/lib/apt/lists/*; \
		rm -f /var/cache/apt/archives/*.deb; \
		rm -f /var/cache/apt/archives/parital/*.deb; \
		rm -f /var/cache/apt/*.bin

RUN mkdir -p /opt
RUN mkdir download; \
		wget https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-${PYTORCH_VERSION}%2Bcpu.zip -O download/libtorch.zip; \
		unzip download/libtorch.zip -d /opt/; \
		rm -r download

ENV LD_LIBRARY_PATH="/usr/local/lib:/opt/libtorch/lib"
ENV Torch_DIR=/opt/libtorch/share/cmake/

COPY requirements_cpu.txt /opt/requirements.txt
RUN python${PYTHON_VERSION} -m venv /opt/venv
RUN /opt/venv/bin/pip install --no-cache-dir wheel
RUN /opt/venv/bin/pip install --no-cache-dir -r /opt/requirements.txt
ENV VENV_PATH=/opt/venv

COPY .bashrc /root/.bashrc
