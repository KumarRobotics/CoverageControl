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
 * \file geographiclib_wrapper.h
 * \brief Wrappers for GeographicLib functions
 * \details This file contains the class GeoLocalTransform which is a wrapper
 * for GeographicLib::LocalCartesian The class has been deprecated and is not
 * used in the current version of the library.
 */

#ifndef CPPSRC_CORE_INCLUDE_COVERAGECONTROL_GEOGRAPHICLIB_WRAPPER_H_
#define CPPSRC_CORE_INCLUDE_COVERAGECONTROL_GEOGRAPHICLIB_WRAPPER_H_

#include <GeographicLib/LocalCartesian.hpp>

#include "CoverageControl/typedefs.h"

namespace CoverageControl {

class GeoLocalTransform {
 private:
  GeographicLib::LocalCartesian geo_obj_;

 public:
  GeoLocalTransform(double const lat, double const lon, double const height) {
    geo_obj_ = GeographicLib::LocalCartesian(lat, lon, height);
  }

  void Reset(double const lat, double const lon, double const height) {
    geo_obj_.Reset(lat, lon, height);
  }

  Point3 Forward(double const lat, double const lon,
                 double const height) const {
    Point3 xyz;
    geo_obj_.Forward(lat, lon, height, xyz[0], xyz[1], xyz[2]);
    return xyz;
  }

  Point3 Reverse(double const x, double const y, double const height) const {
    Point3 lla;
    geo_obj_.Reverse(x, y, height, lla[0], lla[1], lla[2]);
    return lla;
  }
};

} /* namespace CoverageControl */
#endif  // CPPSRC_CORE_INCLUDE_COVERAGECONTROL_GEOGRAPHICLIB_WRAPPER_H_
