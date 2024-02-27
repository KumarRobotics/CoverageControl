FROM ubuntu:22.04

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
											 ca-certificates

# Add repo for installing latest version of cmake
RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null; \
		echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ jammy main' | tee /etc/apt/sources.list.d/kitware.list >/dev/null; \
		apt-get update; \
		rm /usr/share/keyrings/kitware-archive-keyring.gpg

RUN apt install -y kitware-archive-keyring

RUN add-apt-repository -y ppa:deadsnakes/ppa; apt-get update; apt-get upgrade

RUN apt-get -y install \
											 cmake \
											 libgmp-dev \
											 libmpfr-dev \
											 libboost-all-dev \
											 libeigen3-dev \
											 python3.11 \
											 python3.11-dev \
											 python3.11-venv \
											 python-is-python3 \
											 libgeos-dev \
											 libyaml-cpp-dev \
											 vim \
											 tmux \
											 ffmpeg \
											 gnuplot-nox \
											 ninja-build libpng-dev libjpeg-dev libopencv-dev python3-opencv

# Remove cache to reduce image size
RUN rm -rf /var/lib/apt/lists/*; \
		rm -f /var/cache/apt/archives/*.deb; \
		rm -f /var/cache/apt/archives/parital/*.deb; \
		rm -f /var/cache/apt/*.bin

COPY requirements_cpu.txt /opt/requirements.txt
RUN python3.11 -m venv /opt/venv
RUN /opt/venv/bin/pip install --no-cache-dir wheel
RUN /opt/venv/bin/pip install --no-cache-dir -r /opt/requirements.txt
ENV VENV_PATH /opt/venv

ENV LD_LIBRARY_PATH /usr/local/lib:${LD_LIBRARY_PATH}
COPY .bashrc /root/.bashrc
RUN echo "source /opt/venv/bin/activate" >> /root/.bashrc
