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
 * \file generate_world_map.cu
 * \brief File for generating the world map IDF using CUDA
 */

/*
 * RTX 2080 Ti: CUDA Cores 4352, compute capablity: 7.5
 * 68 SMs, 64 CUDA Cores/SM
 * Block size: 32 x 32 = 1024 threads
 * mean_x, mean_y, sigma, scale, rho: 5 * # of distributions
 * resolution, size: 2
 * map: pWorldMapSize * pWorldMapSize
 */

#include <cuda_runtime.h>
#include <thrust/device_ptr.h>
#include <thrust/extrema.h>

#include <cmath>
#include <sstream>

#include "CoverageControl/constants.h"
#include "CoverageControl/cuda/geometry_utils.cuh"
#include "CoverageControl/extern/cuda_helpers/helper_cuda.h"
#include "CoverageControl/generate_world_map.h"

namespace CoverageControl {
__device__ __constant__ int cu_num_dists;
__device__ __constant__ int cu_map_size;
__device__ __constant__ float cu_resolution;
__device__ __constant__ float cu_truncation;
__device__ __constant__ float cu_trun_sq;
__device__ __constant__ float cu_OneBySqrt2;
__device__ __constant__ float cu_normalization_factor;
__device__ __constant__ int cu_polygons_num_pts;
__device__ __constant__ int cu_num_polygons;

__device__ float2 TransformPoint(BND_Cuda const *device_dists, int i,
    float2 const &in_point) {
  float2 pt;
  auto bnd = device_dists[i];
  if (bnd.rho == 0) {
    pt.x = (in_point.x - bnd.mean_x) / bnd.sigma_x;
    pt.y = (in_point.y - bnd.mean_y) / bnd.sigma_y;
    return pt;
  }
  pt.x = (in_point.x - bnd.mean_x) / bnd.sigma_x;
  pt.y = (in_point.y - bnd.mean_y) / bnd.sigma_y;
  pt.x = (pt.x - bnd.rho * pt.y) / (sqrt(1 - bnd.rho * bnd.rho));
  return pt;
}

__device__ float IntegrateQuarterPlane(BND_Cuda const &bnd,
    float2 const &in_point) {
  float2 pt;
  if (bnd.rho == 0) {
    pt.x = (in_point.x - bnd.mean_x) / bnd.sigma_x;
    pt.y = (in_point.y - bnd.mean_y) / bnd.sigma_y;
  } else {
    pt.x = (in_point.x - bnd.mean_x) / bnd.sigma_x;
    pt.y = (in_point.y - bnd.mean_y) / bnd.sigma_y;
    pt.x = (pt.x - bnd.rho * pt.y) / (sqrt(1 - bnd.rho * bnd.rho));
  }
  /* auto transformed_point = TransformPoint(i, in_point); */
  float sc = bnd.scale;
  /* return sc; */
  return sc * erfc(pt.x * cu_OneBySqrt2) * erfc(pt.y * cu_OneBySqrt2) / 4.f;
}

__device__ float ComputeImportanceBND(BND_Cuda const *device_dists,
    float2 const &bottom_left,
    float2 const &top_right,
    float2 const &mid_pt_cell) {
  float2 bottom_right = make_float2(top_right.x, bottom_left.y);
  float2 top_left = make_float2(bottom_left.x, top_right.y);

  float total_importance = 0;
  for (int i = 0; i < cu_num_dists; ++i) {
    auto const &bnd = device_dists[i];
    float2 mid_pt;
    if (bnd.rho == 0) {
      mid_pt.x = (mid_pt_cell.x - bnd.mean_x) / bnd.sigma_x;
      mid_pt.y = (mid_pt_cell.y - bnd.mean_y) / bnd.sigma_y;
    } else {
      mid_pt.x = (mid_pt_cell.x - bnd.mean_x) / bnd.sigma_x;
      mid_pt.y = (mid_pt_cell.y - bnd.mean_y) / bnd.sigma_y;
      mid_pt.x =
        (mid_pt.x - bnd.rho * mid_pt.y) / (sqrt(1 - bnd.rho * bnd.rho));
    }
    if (mid_pt.x * mid_pt.x + mid_pt.y * mid_pt.y > cu_trun_sq) {
      continue;
    }
    total_importance += IntegrateQuarterPlane(bnd, bottom_left);
    total_importance -= IntegrateQuarterPlane(bnd, bottom_right);
    total_importance -= IntegrateQuarterPlane(bnd, top_left);
    total_importance += IntegrateQuarterPlane(bnd, top_right);
  }
  return total_importance;
}

__device__ float ComputeImportancePoly(Polygons_Cuda const &device_polygons,
    float2 const &mid_pt_cell) {
  float total_importance = 0;
  int start = 0;
  auto &x = device_polygons.x;
  auto &y = device_polygons.y;
  for (int i = 0; i < cu_num_polygons; ++i) {
    auto const &bounds = device_polygons.bounds[i];
    if ((mid_pt_cell.x < bounds.xmin) || (mid_pt_cell.x > bounds.xmax) ||
        (mid_pt_cell.y < bounds.ymin) || (mid_pt_cell.y > bounds.ymax)) {
      start += device_polygons.sz[i];
      continue;
    }
    int sz = device_polygons.sz[i];
    if (IsPointInMonotonePolygon(&x[start], &y[start], sz, mid_pt_cell)) {
      total_importance = fmaxf(total_importance, device_polygons.imp[i]);
    }
    start += sz;
  }
  return total_importance;
}

__global__ void kernel(BND_Cuda const *device_dists,
    Polygons_Cuda const device_polygons,
    float *importance_vec) {
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  int idy = blockIdx.y * blockDim.y + threadIdx.y;
  int vec_idx = idx * cu_map_size + idy;
  if (not(idx < cu_map_size and idy < cu_map_size)) {
    return;
  }
  float2 bottom_left = make_float2(idx * cu_resolution, idy * cu_resolution);
  float2 top_right = make_float2(idx * cu_resolution + cu_resolution,
      idy * cu_resolution + cu_resolution);
  float2 mid_pt_cell = make_float2((bottom_left.x + top_right.x) / 2.,
      (bottom_left.y + top_right.y) / 2.);
  float poly_importance = ComputeImportancePoly(device_polygons, mid_pt_cell);
  importance_vec[vec_idx] =
    ComputeImportanceBND(device_dists, bottom_left, top_right, mid_pt_cell) +
    poly_importance;
}

__global__ void normalize(float *importance_vec) {
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  int idy = blockIdx.y * blockDim.y + threadIdx.y;
  int vec_idx = idx * cu_map_size + idy;
  if (not(idx < cu_map_size and idy < cu_map_size)) {
    return;
  }
  importance_vec[vec_idx] *= cu_normalization_factor;
}
void generate_world_map_cuda(BND_Cuda *host_dists,
    Polygons_Cuda_Host const &host_polygons,
    int const num_dists, int const map_size,
    float const resolution, float const truncation,
    float const pNorm, float *host_importance_vec,
    float &normalization_factor) {
  checkCudaErrors(cudaDeviceSynchronize());

  checkCudaErrors(cudaMemcpyToSymbol(cu_num_dists, &num_dists, sizeof(int)));
  checkCudaErrors(cudaMemcpyToSymbol(cu_map_size, &map_size, sizeof(int)));
  checkCudaErrors(
      cudaMemcpyToSymbol(cu_resolution, &resolution, sizeof(float)));
  checkCudaErrors(
      cudaMemcpyToSymbol(cu_truncation, &truncation, sizeof(float)));
  float trun_sq = truncation * truncation;
  checkCudaErrors(cudaMemcpyToSymbol(cu_trun_sq, &trun_sq, sizeof(float)));
  float f_OneBySqrt2 = static_cast<float>(1. / std::sqrt(2.));
  checkCudaErrors(
      cudaMemcpyToSymbol(cu_OneBySqrt2, &f_OneBySqrt2, sizeof(float)));

  BND_Cuda *device_dists;
  checkCudaErrors(cudaMalloc(&device_dists, num_dists * sizeof(BND_Cuda)));
  checkCudaErrors(cudaMemcpy(device_dists, host_dists,
        num_dists * sizeof(BND_Cuda),
        cudaMemcpyHostToDevice));

  Polygons_Cuda device_polygons;
  checkCudaErrors(
      cudaMalloc(&(device_polygons.x), host_polygons.num_pts * sizeof(float)));
  checkCudaErrors(
      cudaMalloc(&(device_polygons.y), host_polygons.num_pts * sizeof(float)));
  checkCudaErrors(cudaMalloc(&(device_polygons.imp),
        host_polygons.num_polygons * sizeof(float)));
  checkCudaErrors(cudaMalloc(&(device_polygons.sz),
        host_polygons.num_polygons * sizeof(int)));
  checkCudaErrors(cudaMalloc(&(device_polygons.bounds),
        host_polygons.num_polygons * sizeof(Bounds)));

  checkCudaErrors(cudaMemcpy(device_polygons.x, host_polygons.x.data(),
        host_polygons.num_pts * sizeof(float),
        cudaMemcpyHostToDevice));
  checkCudaErrors(cudaMemcpy(device_polygons.y, host_polygons.y.data(),
        host_polygons.num_pts * sizeof(float),
        cudaMemcpyHostToDevice));
  checkCudaErrors(cudaMemcpy(device_polygons.imp, host_polygons.imp.data(),
        host_polygons.num_polygons * sizeof(float),
        cudaMemcpyHostToDevice));
  checkCudaErrors(cudaMemcpy(device_polygons.sz, host_polygons.sz.data(),
        host_polygons.num_polygons * sizeof(int),
        cudaMemcpyHostToDevice));
  checkCudaErrors(cudaMemcpy(
        device_polygons.bounds, host_polygons.bounds.data(),
        host_polygons.num_polygons * sizeof(Bounds), cudaMemcpyHostToDevice));
  checkCudaErrors(cudaMemcpyToSymbol(cu_polygons_num_pts,
        &host_polygons.num_pts, sizeof(int)));
  checkCudaErrors(cudaMemcpyToSymbol(cu_num_polygons,
        &host_polygons.num_polygons, sizeof(int)));

  float *device_importance_vec;
  checkCudaErrors(
      cudaMalloc(&device_importance_vec, map_size * map_size * sizeof(float)));

  /* dim3 dimBlock(1, 1, 1); */
  /* dim3 dimGrid(1,1,1); */

  dim3 dimBlock(32, 32, 1);
  dim3 dimGrid(map_size / dimBlock.x, map_size / dimBlock.x, 1);

  kernel<<<dimGrid, dimBlock>>>(device_dists, device_polygons,
      device_importance_vec);

  cudaDeviceSynchronize();

  thrust::device_ptr<float> d_ptr =
    thrust::device_pointer_cast(device_importance_vec);
  float max = *(thrust::max_element(d_ptr, d_ptr + map_size * map_size));

  if (max < kEps) {
    normalization_factor = pNorm;
  } else {
    normalization_factor = pNorm / max;
  }
  if (normalization_factor > 1e-5) {
    checkCudaErrors(cudaMemcpyToSymbol(cu_normalization_factor,
          &normalization_factor, sizeof(float)));
    normalize<<<dimGrid, dimBlock>>>(device_importance_vec);
  }

  checkCudaErrors(cudaMemcpy(host_importance_vec, device_importance_vec,
        map_size * map_size * sizeof(float),
        cudaMemcpyDeviceToHost));

  checkCudaErrors(cudaFree(device_dists));
  checkCudaErrors(cudaFree(device_importance_vec));
  checkCudaErrors(cudaFree(device_polygons.x));
  checkCudaErrors(cudaFree(device_polygons.y));
  checkCudaErrors(cudaFree(device_polygons.sz));
  checkCudaErrors(cudaFree(device_polygons.bounds));

  cudaError_t error = cudaGetLastError();
  if (error != cudaSuccess) {
    std::stringstream strstr;
    strstr << "run_kernel launch failed" << std::endl;
    throw strstr.str();
  }
}
} /* namespace CoverageControl */
