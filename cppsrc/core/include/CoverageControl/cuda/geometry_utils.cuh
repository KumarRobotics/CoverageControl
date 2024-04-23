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
 * \file geometry_utils.cuh
 * \brief CUDA supported functions to check whether a point is inside a monotone
 * polygon
 */

#ifndef CPPSRC_CORE_INCLUDE_COVERAGECONTROL_CUDA_GEOMETRY_UTILS_CUH_
#define CPPSRC_CORE_INCLUDE_COVERAGECONTROL_CUDA_GEOMETRY_UTILS_CUH_

#include <cuda_runtime.h>

/* #include "CoverageControl/extern/cuda_helpers/helper_cuda.h" */

namespace CoverageControl {
/*!
 * Get the orientation of the point r with respect to the directional vector
 * from p to q \return 1 if point r is in counter clockwise direction, i.e.,
 * left of the vector \return -1 if point r is in clockwise direction, i.e.,
 * right of the vector \return  0 otherwise
 */
__host__ __device__ int Orientation(float2 const &p, float2 const &q,
                                    float2 const &r) {
  float2 qp{q.x - p.x, q.y - p.y};
  float2 rp{r.x - p.x, r.y - p.y};
  float cross_prod1 = qp.x * rp.y;
  float cross_prod2 = qp.y * rp.x;
  if (cross_prod1 > cross_prod2) return 1;
  if (cross_prod1 < cross_prod2) return -1;
  return 0;
}

/*!
 * Check if the point r is inside the monotone polygon defined by the vertices x
 * and y \return true if the point is inside the polygon \return false otherwise
 */
__host__ __device__ bool IsPointInMonotonePolygon(float *x, float *y, int sz,
                                                  float2 const &r) {
  bool left = false;
  bool right = false;
  for (int ct = 0; ct < sz; ++ct) {
    int ctp1 = ct + 1;
    if (ctp1 == sz) {
      ctp1 = 0;
    }
    float yct = y[ct];
    float yctp1 = y[ctp1];
    /* printf("Edge: %f, %f, %f, %f\n", x[ct], yct, x[ctp1], yctp1); */
    float2 p, q;
    if (yct < yctp1) {
      if (r.y < yct or r.y > yctp1) {
        continue;
      }
      p = float2{x[ct], yct};
      q = float2{x[ctp1], yctp1};
    } else {
      if (r.y < yctp1 or r.y > yct) {
        continue;
      }
      q = float2{x[ct], yct};
      p = float2{x[ctp1], yctp1};
    }
    int orientation = Orientation(p, q, r);
    /* printf("orientation: %d\n", orientation); */
    if (orientation == 0) {
      return true;
    }
    if (orientation == 1) {
      left = true;
    }
    if (orientation == -1) {
      right = true;
    }
    if ((left & right)) {
      return true;
    }
  }
  return false;
}
} /* namespace CoverageControl */

#endif  // CPPSRC_CORE_INCLUDE_COVERAGECONTROL_CUDA_GEOMETRY_UTILS_CUH_
