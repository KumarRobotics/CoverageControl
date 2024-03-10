#!/bin/env bash

yum install -y epel-release
yum install -y wget yum-utils python3-devel

yum-config-manager --add-repo https://developer.download.nvidia.com/compute/cuda/repos/rhel7/x86_64/cuda-rhel7.repo
yum install --setopt=obsoletes=0 -y \
	cuda-nvcc-12-1-12.1.105-1 \
	cuda-cudart-devel-12-1-12.1.105-1 \
	libcurand-devel-12-1-10.3.2.106-1 \
	libcudnn8-devel-8.9.3.28-1.cuda12.1 \
	libcublas-devel-12-1-12.1.3.1-1 \
	libnccl-devel-2.18.3-1+cuda12.1

export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH

bash setup_utils/install_dependencies.sh --boost --gmp --mpfr --eigen --cgal
