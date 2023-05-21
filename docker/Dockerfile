# Start with cuda image
FROM nvidia/cuda:11.8.0-devel-ubuntu22.04

CMD ["/bin/bash"]
# Set the working directory to the user home (user is root).

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
											 gdb

# Add repo for installing latest version of cmake
RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null; \
		echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ jammy main' | tee /etc/apt/sources.list.d/kitware.list >/dev/null; \
		apt-get update; \
		rm /usr/share/keyrings/kitware-archive-keyring.gpg

RUN apt-get -y install \
											 kitware-archive-keyring \
											 cmake

# Install rest of the dependencies
RUN apt-get -y install \
											 libboost-dev \
											 libboost-all-dev \
											 libcgal-dev \
											 python3-dev \
											 python3-venv \
											 python-is-python3 \
											 libgeos-dev \
											 vim \
											 tmux \
											 ffmpeg \
											 gnuplot-nox \
											 texlive-base texlive-latex-extra texlive-science \
											 dvipng texlive-fonts-recommended cm-super

RUN apt-get -y install \
											ninja-build libpng-dev libjpeg-dev

# Remove cache to reduce image size
RUN rm -rf /var/lib/apt/lists/*; \
		rm -f /var/cache/apt/archives/*.deb; \
		rm -f /var/cache/apt/archives/parital/*.deb; \
		rm -f /var/cache/apt/*.bin

ENV COVERAGECONTROL_WS /root/CoverageControl_ws
ENV LD_LIBRARY_PATH ${COVERAGECONTROL_WS}/install/lib:${LD_LIBRARY_PATH}

ENV VIRTUAL_ENV /root/venv
COPY requirements.txt /root/requirements.txt
RUN python -m venv /root/venv
RUN /root/venv/bin/pip install --no-cache-dir -r /root/requirements.txt

RUN mkdir download
RUN wget https://download.pytorch.org/libtorch/cu118/libtorch-cxx11-abi-shared-with-deps-2.0.1%2Bcu118.zip -O download/libtorch.zip
RUN unzip download/libtorch.zip -d /root/
RUN rm -r download

ENV LD_LIBRARY_PATH /usr/local/lib:/root/libtorch/lib:${LD_LIBRARY_PATH}
ENV Torch_DIR /root/libtorch/share/cmake/

RUN mkdir -p /root/dependencies/build
RUN mkdir -p /root/dependencies/src
COPY install_dependencies.sh /root/dependencies/install_dependencies.sh
RUN /root/dependencies/install_dependencies.sh /root/dependencies
RUN rm -r /root/dependencies

ENV PATH ${COVERAGECONTROL_WS}/install/bin:${PATH}
RUN echo "source /root/venv/bin/activate" >> /root/.bashrc

WORKDIR ${COVERAGECONTROL_WS}
