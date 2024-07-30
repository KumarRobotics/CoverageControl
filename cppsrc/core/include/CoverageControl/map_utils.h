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
 * \file map_utils.h
 * \brief Utility functions for transforming maps
 */

#ifndef CPPSRC_CORE_INCLUDE_COVERAGECONTROL_MAP_UTILS_H_
#define CPPSRC_CORE_INCLUDE_COVERAGECONTROL_MAP_UTILS_H_

#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

#include "CoverageControl/typedefs.h"

namespace CoverageControl {
/*!
 * \brief Namespace for map utilities functions
 */
namespace MapUtils {
struct MapBounds {
  int left = 0, right = 0, bottom = 0, top = 0;
  int width = 0, height = 0;
  void SetZero() { left = 0, right = 0, bottom = 0, top = 0; }
};

//! Gets the closest point on the grid
inline void GetClosestGridCoordinate(double const resolution, Point2 const &pt,
                                     int &idx, int &idy) {
  idx = std::round(pt.x() / resolution);
  idy = std::round(pt.y() / resolution);
}

//! Compute necessary map transformations when the point is close to the
//! boundary
inline void ComputeOffsets(double const resolution, Point2 const &pos,
                           int const submap_size, int const map_size,
                           MapBounds &index, MapBounds &offset) {
  int pos_idx = 0, pos_idy = 0;
  GetClosestGridCoordinate(resolution, pos, pos_idx, pos_idy);
  index.left = pos_idx - submap_size / 2;
  index.right = pos_idx + submap_size / 2;
  index.bottom = pos_idy - submap_size / 2;
  index.top = pos_idy + submap_size / 2;

  offset.SetZero();
  if (index.left < 0) {
    offset.left = -index.left;
  }
  if (index.bottom < 0) {
    offset.bottom = -index.bottom;
  }

  if (index.right > map_size) {
    offset.right = map_size - index.right;
  }
  if (index.top > map_size) {
    offset.top = map_size - index.top;
  }

  offset.width = index.right + offset.right - (index.left + offset.left);
  offset.height = index.top + offset.top - (index.bottom + offset.bottom);
}

template <typename T = MapType>
inline void GetSubMap(double const resolution, Point2 const &pos,
                      int const map_size, T const &map, int const submap_size,
                      T &submap) {
  MapBounds index, offset;
  ComputeOffsets(resolution, pos, submap_size, map_size, index, offset);
  submap.block(offset.left, offset.bottom, offset.width, offset.height) =
      map.block(index.left + offset.left, index.bottom + offset.bottom,
                offset.width, offset.height);
}

template <typename T = MapType>
inline auto GetSubMap(double const resolution, Point2 const &pos,
                      int const map_size, T const &map, int const submap_size) {
  MapBounds index, offset;
  ComputeOffsets(resolution, pos, submap_size, map_size, index, offset);
  return map.block(index.left + offset.left, index.bottom + offset.bottom,
                   offset.width, offset.height);
}

//! Write the world map to a file
inline int WriteMap(MapType const &map, std::string const &file_name) {
  std::ofstream file_obj(file_name);
  if (!file_obj) {
    std::cerr << "[Error] Could not open " << file_name << " for writing."
              << std::endl;
    return 1;
  }
  file_obj << map;
  file_obj.close();
  return 0;
}

//! Write the world map to a file
inline int WriteMapSparse(MapType const &map, std::string const &file_name) {
  std::ofstream file_obj(file_name);
  if (!file_obj) {
    std::cerr << "[Error] Could not open " << file_name << " for writing."
              << std::endl;
    return 1;
  }
  for (int i = 0; i < map.rows(); ++i) {
    for (int j = 0; j < map.cols(); ++j) {
      if (map(i, j) >= 0) {
        file_obj << i << " " << j << " " << map(i, j) << std::endl;
      }
    }
  }
  file_obj.close();
  return 0;
}

inline int IsPointOutsideBoundary(double const resolution, Point2 const &pos,
                                  int const sensor_size, int const boundary) {
  if (pos.x() <= -sensor_size * resolution / 2.) {
    return 1;
  }
  if (pos.y() <= -sensor_size * resolution / 2.) {
    return 1;
  }
  if (pos.x() >= boundary * resolution + sensor_size * resolution / 2.) {
    return 1;
  }
  if (pos.y() >= boundary * resolution + sensor_size * resolution / 2.) {
    return 1;
  }
  return 0;
}

} /* namespace MapUtils */
} /* namespace CoverageControl */
#endif  // CPPSRC_CORE_INCLUDE_COVERAGECONTROL_MAP_UTILS_H_
