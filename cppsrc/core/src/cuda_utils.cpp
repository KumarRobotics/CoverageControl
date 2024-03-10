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
 * \file cuda_utils.cpp
 */

#include "CoverageControl/cuda_utils.h"

namespace CoverageControl {

bool CudaUtils::is_cuda_initialized_ = false;
bool CudaUtils::is_cuda_available_ = false;
int CudaUtils::device_count_ = 0;
int CudaUtils::device_id_ = -1;
#ifdef COVERAGECONTROL_WITH_CUDA
bool CudaUtils::use_cuda_ = true;
#else
bool CudaUtils::use_cuda_ = false;
#endif

}  // namespace CoverageControl
