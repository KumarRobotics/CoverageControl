#!/bin/env bash

yum install -y epel-release
yum install -y wget yum-utils python3-devel

dnf config-manager --add-repo https://developer.download.nvidia.com/compute/cuda/repos/rhel8/x86_64/cuda-rhel8.repo
# CUDA <= 12.6 nvcc rejects the GCC 14 toolchain in current manylinux_2_28
# images; 12.8 is the earliest that accepts it and matches PyTorch's cu128
# builds. The meta packages cover nvcc, cudart-devel, curand-devel, and
# cublas-devel; cuDNN/NCCL are not linked by the core library.
dnf -y install --setopt=obsoletes=0 \
  cuda-compiler-12-8 \
  cuda-libraries-12-8 \
  cuda-libraries-devel-12-8

export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
export CUDA_HOME=/usr/local/cuda
export CUDA_ROOT=/usr/local/cuda
export CUDA_PATH=/usr/local/cuda
export CUDADIR=/usr/local/cuda

bash utils/setup/install_dependencies.sh --boost --gmp --mpfr --cgal
