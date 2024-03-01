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

#include <CoverageControl/geographiclib_wrapper.h>

#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>

#include <GeographicLib/LocalCartesian.hpp>

int main(int argc, char** argv) {
  CoverageControl::GeoLocalTransform geo_transform(40.74050005471615,
                                                   -74.1759877275644, 0);
  auto lla = geo_transform.Reverse(100, 100, 0);
  std::cout << "lat: " << lla[0] << std::endl;
  std::cout << "lon: " << lla[1] << std::endl;
  std::cout << "hgt: " << lla[2] << std::endl;

  auto xyz = geo_transform.Forward(lla[0], lla[1], lla[2]);
  std::cout << "x: " << xyz[0] << std::endl;
  std::cout << "y: " << xyz[1] << std::endl;
  std::cout << "z: " << xyz[2] << std::endl;

  GeographicLib::LocalCartesian geo_transform1(40.74050005471615,
                                               -74.1759877275644);
  geo_transform1.Forward(lla[0], lla[1], lla[2], xyz[0], xyz[1], xyz[2]);
  std::cout << "x: " << xyz[0] << std::endl;
  std::cout << "y: " << xyz[1] << std::endl;
  std::cout << "z: " << xyz[2] << std::endl;

  return 0;
}
