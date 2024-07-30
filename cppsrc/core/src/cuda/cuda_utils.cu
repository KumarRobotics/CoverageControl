/*
 * This file is part of the CoverageControl library
 *
 * Author: Saurav Agarwal
 * Contact: sauravag@seas.upenn.edu, agr.saurav1@gmail.com
 * Repository: https://github.com/KumarRobotics/CoverageControl
 *
 * Copyright (c) 2024, Saurav Agarwal
 *
 * The CoverageControl library is free software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * The CoverageControl library is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * CoverageControl library. If not, see <https://www.gnu.org/licenses/>.
 */

/*!
 * \file cuda_utils.cu
 * \brief Functions for CUDA utilities based on NVIDIA's CUDA Helper Library
 * See: extern/cuda_helpers/helper_cuda.h
 */

#include <cuda_runtime.h>

#include <iostream>

#include "CoverageControl/cuda_utils.h"
#include "CoverageControl/extern/cuda_helpers/helper_cuda.h"
#include "CoverageControl/extern/cuda_helpers/helper_string.h"

namespace CoverageControl {

bool CheckCudaErrors(cudaError_t result) {
  if (result != cudaSuccess) {
    std::cerr << "CUDA error: [" << static_cast<unsigned int>(result) << "] "
      << cudaGetErrorString(result) << std::endl;
    return false;
  }
  return true;
}

bool CudaUtils::GetDeviceCount(int &device_count) {
  CheckCudaErrors(cudaGetDeviceCount(&device_count));
  if (device_count == 0) {
    /* std::cerr << "No CUDA devices found" << std::endl; */
    return false;
  }
  return true;
}

int CudaUtils::GPUDeviceInit(int dev_id) {
  int device_count;
  bool device_count_success = GetDeviceCount(device_count);
  if (!device_count_success) {
    return -1;
  }

  if (dev_id < 0) {
    dev_id = 0;
  }

  if (dev_id > device_count - 1) {
    std::cerr << "Invalid GPU device ID" << std::endl;
    std::cerr << "Device ID: " << dev_id << " Device Count: " << device_count
      << std::endl;
    return -1;
  }

  int compute_mode = -1, major = 0, minor = 0;
  CheckCudaErrors(
      cudaDeviceGetAttribute(&compute_mode, cudaDevAttrComputeMode, dev_id));
  CheckCudaErrors(cudaDeviceGetAttribute(
        &major, cudaDevAttrComputeCapabilityMajor, dev_id));
  CheckCudaErrors(cudaDeviceGetAttribute(
        &minor, cudaDevAttrComputeCapabilityMinor, dev_id));
  if (compute_mode == cudaComputeModeProhibited) {
    std::cerr << "Error: device is running in <Compute Mode Prohibited>, no "
      "threads can use cudaSetDevice()."
      << std::endl;
    return -1;
  }

  if (major < 1) {
    std::cerr << "GPUDeviceInit(): GPU device does not support CUDA."
      << std::endl;
    return -1;
  }

  CheckCudaErrors(cudaSetDevice(dev_id));
  /* std::cout << "GPU Device " << dev_id << " has been set" << std::endl; */
  /* std::cout << "CUDA Device [" << dev_id << "]: \"" */
    /* << _ConvertSMVer2ArchName(major, minor) << "\"" << std::endl; */
  is_cuda_initialized_ = true;
  device_id_ = dev_id;
  return dev_id;
}

int CudaUtils::FindDevice() { return GPUGetMaxGflopsDeviceId(); }

int CudaUtils::FindIntegratedGPU() {
  int current_device = 0;
  int device_count = 0;
  int devices_prohibited = 0;

  CheckCudaErrors(cudaGetDeviceCount(&device_count));

  if (device_count == 0) {
    /* std::cerr << "No CUDA devices found" << std::endl; */
    return -1;
  }

  // Find the integrated GPU which is compute capable
  while (current_device < device_count) {
    int compute_mode = -1, integrated = -1;
    CheckCudaErrors(cudaDeviceGetAttribute(
          &compute_mode, cudaDevAttrComputeMode, current_device));
    CheckCudaErrors(cudaDeviceGetAttribute(&integrated, cudaDevAttrIntegrated,
          current_device));
    // If GPU is integrated and is not running on Compute Mode prohibited,
    // then cuda can map to GLES resource
    if (integrated && (compute_mode != cudaComputeModeProhibited)) {
      CheckCudaErrors(cudaSetDevice(current_device));

      int major = 0, minor = 0;
      CheckCudaErrors(cudaDeviceGetAttribute(
            &major, cudaDevAttrComputeCapabilityMajor, current_device));
      CheckCudaErrors(cudaDeviceGetAttribute(
            &minor, cudaDevAttrComputeCapabilityMinor, current_device));
      /* std::cout << "GPU Device " << current_device << " has been set" */
      /*   << std::endl; */
      /* std::cout << "CUDA Device [" << current_device << "]: \"" */
      /*   << _ConvertSMVer2ArchName(major, minor) << "\"" << std::endl; */
      return current_device;
    } else {
      devices_prohibited++;
    }

    current_device++;
  }

  if (devices_prohibited == device_count) {
    std::cerr << "CUDA error: No Integrated GPU found that supports CUDA."
      << std::endl;
    return -1;
  }

  return -1;
}

/*!
 * \brief Provides a way to select the best GPU based on maximum GFLOPS
 * \return The device ID of the GPU with the maximum GFLOPS
 * \return -1 if no device is found
 * See: extern/cuda_helpers/helper_cuda.h
 */
int CudaUtils::GPUGetMaxGflopsDeviceId(std::vector<int> device_list) {
  int sm_per_multiproc = 0;
  int max_perf_device = 0;
  int device_count = 0;
  int devices_prohibited = 0;

  uint64_t max_compute_perf = 0;

  if (GetDeviceCount(device_count) == false) {
    return -1;
  }

  if (device_count == 0) {
    /* std::cerr << "No CUDA devices found" << std::endl; */
    return -1;
  }

  if (device_list.size() == 0) {
    for (int i = 0; i < device_count; i++) {
      device_list.push_back(i);
    }
  }

  while (device_list.size() > 0) {
    int current_device = device_list.back();
    device_list.pop_back();
    int compute_mode = -1, major = 0, minor = 0;
    CheckCudaErrors(cudaDeviceGetAttribute(
          &compute_mode, cudaDevAttrComputeMode, current_device));
    CheckCudaErrors(cudaDeviceGetAttribute(
          &major, cudaDevAttrComputeCapabilityMajor, current_device));
    CheckCudaErrors(cudaDeviceGetAttribute(
          &minor, cudaDevAttrComputeCapabilityMinor, current_device));

    // If this GPU is not running on Compute Mode prohibited,
    // then we can add it to the list
    if (compute_mode != cudaComputeModeProhibited) {
      if (major == 9999 && minor == 9999) {
        sm_per_multiproc = 1;
      } else {
        sm_per_multiproc = _ConvertSMVer2Cores(major, minor);
      }
      int multiProcessorCount = 0, clockRate = 0;
      CheckCudaErrors(cudaDeviceGetAttribute(&multiProcessorCount,
            cudaDevAttrMultiProcessorCount,
            current_device));
      cudaError_t result = cudaDeviceGetAttribute(
          &clockRate, cudaDevAttrClockRate, current_device);
      if (result != cudaSuccess) {
        // If cudaDevAttrClockRate attribute is not supported we
        // set clockRate as 1, to consider GPU with most SMs and CUDA Cores.
        if (result == cudaErrorInvalidValue) {
          clockRate = 1;
        } else {
          fprintf(stderr, "CUDA error at %s:%d code=%d(%s) \n", __FILE__,
              __LINE__, static_cast<unsigned int>(result),
              _cudaGetErrorEnum(result));
          return -1;
        }
      }
      uint64_t compute_perf =
        (uint64_t)multiProcessorCount * sm_per_multiproc * clockRate;

      if (compute_perf > max_compute_perf) {
        max_compute_perf = compute_perf;
        max_perf_device = current_device;
      }
    } else {
      devices_prohibited++;
    }
  }

  if (devices_prohibited == device_count) {
    std::cerr << "GPUGetMaxGflopsDeviceId() CUDA error:"
      << " all devices have compute mode prohibited." << std::endl;
    return -1;
  }

  return max_perf_device;
}

} /* namespace CoverageControl */
