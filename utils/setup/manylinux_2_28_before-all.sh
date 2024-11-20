#!/bin/env bash

yum install -y epel-release
yum install -y wget yum-utils python3-devel

dnf config-manager --add-repo https://developer.download.nvidia.com/compute/cuda/repos/rhel8/x86_64/cuda-rhel8.repo
dnf -y install --setopt=obsoletes=0 \
  cuda-compiler-12-4-12.4.1-1 \
  cuda-libraries-12-4-12.4.1-1 \
  cuda-libraries-devel-12-4-12.4.1-1 \
  cuda-nvcc-12-4-12.4.131-1 \
  cuda-cudart-devel-12-4-12.4.127-1 \
  libcurand-devel-12-4-10.3.5.147-1 \
  libcudnn9-devel-cuda-12-9.5.1.17-1 \
  libcublas-devel-12-4-12.4.5.8-1 \
  libnccl-devel-2.23.4-1+cuda12.4

export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
export CUDA_HOME=/usr/local/cuda
export CUDA_ROOT=/usr/local/cuda
export CUDA_PATH=/usr/local/cuda
export CUDADIR=/usr/local/cuda

bash utils/setup/install_dependencies.sh --boost --gmp --mpfr --eigen --cgal
