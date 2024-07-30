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
 * \file cuda_utils.h
 * \brief Utility functions for CUDA
 */

#ifndef CPPSRC_CORE_INCLUDE_COVERAGECONTROL_CUDA_UTILS_H_
#define CPPSRC_CORE_INCLUDE_COVERAGECONTROL_CUDA_UTILS_H_

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "CoverageControl/Config.h"
namespace CoverageControl {

/*!
 * \brief Static class for CUDA utilities
 * This class provides utility functions for CUDA
 * It is a static class and cannot be instantiated
 */
class CudaUtils {
 private:
  static bool is_cuda_initialized_;
  static bool is_cuda_available_;
  static int device_count_;
  static int device_id_;
  static bool use_cuda_;

 public:
  /*!
   * Constructor deleted as we don't want to create an instance of this class
   */
  CudaUtils() = delete;

  static bool UseCuda() { return use_cuda_; }

  static void SetUseCuda(bool use_cuda) { use_cuda_ = use_cuda; }

  /*!
   * Check if CUDA enabled GPU is available
   * \return True if CUDA enabled GPU is available
   */
  static bool IsCudaAvailable() {
#ifdef COVERAGECONTROL_WITH_CUDA
    int device_count;
    return !!GetDeviceCount(device_count);
#endif
    return false;
  }

  /*!
   * Check if CUDA is initialized
   * \return True if CUDA is initialized
   */
  static bool IsCudaInitialized() {
    if (!use_cuda_) {
      return false;
    }
    return is_cuda_initialized_;
  }

  /*!
   * \brief Initializes a CUDA device
   * `use_cuda_` must be set to true before calling this function
   * \return True if successful
   */
  static bool InitializeCUDA() {
#ifdef COVERAGECONTROL_WITH_CUDA
    if (!use_cuda_) {
      return false;
    }
    if (is_cuda_initialized_) {
      return true;
    }
    auto devices = GetEnvironmentCUDA_VISIBLE_DEVICES();
    if (devices.empty()) {
      device_id_ = FindDevice();
    } else {
      /* device_id_ = GPUGetMaxGflopsDeviceId(devices); */
      device_id_ = 0;
    }
    if (device_id_ < 0) {
      std::cerr << "No CUDA device found" << std::endl;
      return false;
    }
    /* std::cout << "Initializing CUDA device " << device_id_ << std::endl; */
    if (GPUDeviceInit(device_id_) != device_id_) {
      std::cerr << "Failed to initialize CUDA device" << std::endl;
      return false;
    }
    is_cuda_initialized_ = true;
    return true;
#endif
    return false;
  }

#ifdef COVERAGECONTROL_WITH_CUDA
  /*!
   * Get the environment variable CUDA_VISIBLE_DEVICES
   * \return Vector of CUDA_VISIBLE_DEVICES
   */
  static std::vector<int> GetEnvironmentCUDA_VISIBLE_DEVICES() {
    std::vector<int> cuda_visible_devices;
    char *env_cuda_visible_devices = std::getenv("CUDA_VISIBLE_DEVICES");
    if (env_cuda_visible_devices != nullptr) {
      std::string str_cuda_visible_devices(env_cuda_visible_devices);
      std::istringstream ss(str_cuda_visible_devices);
      std::string token;
      while (std::getline(ss, token, ',')) {
        cuda_visible_devices.push_back(std::stoi(token));
      }
    }
    return cuda_visible_devices;
  }

  /*!
   * Get the number of CUDA enabled GPUs
   * \param device_count Number of CUDA enabled GPUs
   * \return True if successful
   */
  static bool GetDeviceCount(int &device_count);

  /*!
   * Initialize CUDA device
   * \param dev_id Device ID
   * \return dev_id if successful
   */
  static int GPUDeviceInit(int dev_id);

  /*!
   * Checks if the given command line argument is a CUDA device ID
   * These can be passed to the program using the --device_id flag
   * If no flag is passed, it gets the device with highest Gflops/s
   * \param argc Number of command line arguments
   * \param argv Command line arguments
   * \return Device ID
   */
  static int FindDevice();

  /*!
   * Get the integrated GPU compatible with CUDA
   * \return Device ID of the integrated GPU
   */
  static int FindIntegratedGPU();

  /*!
   * \brief Provides a way to select the best GPU based on maximum GFLOPS
   * \return The device ID of the GPU with the maximum GFLOPS
   * \return -1 if no device is found
   * See: extern/cuda_helpers/helper_cuda.h
   */
  static int GPUGetMaxGflopsDeviceId(std::vector<int> device_list = {});
#endif
};
} /* namespace CoverageControl */

#endif  // CPPSRC_CORE_INCLUDE_COVERAGECONTROL_CUDA_UTILS_H_
