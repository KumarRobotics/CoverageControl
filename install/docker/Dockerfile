# Start with cuda image
FROM nvidia/cuda:11.8.0-devel-ubuntu22.04

# CMD ["/bin/bash"]
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
											 gpg

# Add repo for installing latest version of cmake
RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null; \
		echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ jammy main' | tee /etc/apt/sources.list.d/kitware.list >/dev/null; \
		apt-get update; \
		rm /usr/share/keyrings/kitware-archive-keyring.gpg

RUN add-apt-repository -y ppa:deadsnakes/ppa; apt-get update; apt-get upgrade

RUN apt-get -y install \
											 cmake \
											 libboost-dev \
											 libboost-all-dev \
											 libcgal-dev \
											 python3.11 \
											 python3.11-dev \
											 python3.11-venv \
											 python-is-python3 \
											 python3-numpy \
											 libgeos-dev \
											 vim \
											 tmux \
											 ffmpeg \
											 gnuplot-nox \
											 texlive-base texlive-latex-extra texlive-science \
											 dvipng texlive-fonts-recommended cm-super \
											 ninja-build libpng-dev libjpeg-dev libopencv-dev python3-opencv

# Remove cache to reduce image size
RUN rm -rf /var/lib/apt/lists/*; \
		rm -f /var/cache/apt/archives/*.deb; \
		rm -f /var/cache/apt/archives/parital/*.deb; \
		rm -f /var/cache/apt/*.bin

COPY requirements.txt /root/requirements.txt
RUN python3.11 -m venv /root/venv
RUN /root/venv/bin/pip install --no-cache-dir -r /root/requirements.txt

RUN mkdir download; \
		wget https://download.pytorch.org/libtorch/cu118/libtorch-cxx11-abi-shared-with-deps-2.0.1%2Bcu118.zip -O download/libtorch.zip; \
		unzip download/libtorch.zip -d /root/; \
		rm -r download

ENV LD_LIBRARY_PATH /usr/local/lib:/root/libtorch/lib:${LD_LIBRARY_PATH}
ENV Torch_ROOT /root/libtorch/
ENV Torch_DIR /root/libtorch/share/cmake/

RUN mkdir -p /root/dependencies/build; \
		mkdir -p /root/dependencies/src
COPY install_dependencies.sh /root/dependencies/install_dependencies.sh
RUN ["/bin/bash", "-c", "/root/dependencies/install_dependencies.sh /root/dependencies"]
RUN rm -r /root/dependencies

ENV TorchVision_DIR /root/libtorch/share/cmake/

ENV COVERAGECONTROL_WS /root/CoverageControl_ws
ENV LD_LIBRARY_PATH ${COVERAGECONTROL_WS}/install/lib:${LD_LIBRARY_PATH}
ENV PATH ${COVERAGECONTROL_WS}/install/bin:${PATH}
ENV LIBTORCH_PYTHON_PATH /root/libtorch/lib/libtorch_python.so
COPY .bashrc /root/.bashrc
RUN echo "source /root/venv/bin/activate" >> /root/.bashrc

RUN mkdir -p ${COVERAGECONTROL_WS}
WORKDIR ${COVERAGECONTROL_WS}
