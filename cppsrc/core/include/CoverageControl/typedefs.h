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
 * \file typedefs.h
 * \brief Contains typedefs for the library
 */

#ifndef CPPSRC_CORE_INCLUDE_COVERAGECONTROL_TYPEDEFS_H_
#define CPPSRC_CORE_INCLUDE_COVERAGECONTROL_TYPEDEFS_H_

#define EIGEN_NO_CUDA   // Don't use eigen's cuda facility
#include <Eigen/Dense>  // Eigen is used for maps
#include <queue>
#include <vector>

namespace CoverageControl {

/*!
 * \addtogroup cpp_api_typedefs Typedefs
 * @{
 */

typedef Eigen::Vector2d Point2;  /*!< Point2 is a 2D vector of doubles */
typedef Eigen::Vector2f Point2f; /*!< Point2 is a 2D vector of doubles */
typedef Eigen::Vector3d Point3;  /*!< Point3 is a 3D vector of doubles */
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
    MapType; /*!< MapType is a 2D matrix of floats. Note: It is RowMajor */

typedef std::vector<Point2>
    PointVector; /*!< PointVector is a vector of 2D points */

/*!
 * \brief A struct to store a polygon feature and a uniform importance value
 */
struct PolygonFeature {
  PointVector poly;
  float imp = 0; /*!< Importance of the polygon */
  int size = 0;  /*!< Number of vertices in the polygon */
  PolygonFeature() {}
  PolygonFeature(PointVector const &p, float const i) : poly{p}, imp{i} {
    size = poly.size();
  }
};

/*!
 * \brief A struct to store a frontier points and a value
 */
struct Frontier {
  Point2 pt;
  double value;
  Frontier() : pt{Point2()}, value{0} {}
  Frontier(Point2 const &p, double const &v) : pt{p}, value{v} {}
};

/*!
 * \brief A struct to compare frontiers based on their value
 */
struct FrontierCompare {
  bool operator()(Frontier const &left, Frontier const &right) {
    return left.value > right.value;
  }
};

typedef std::priority_queue<Frontier, std::vector<Frontier>, FrontierCompare>
    queue_t; /*!< A priority queue of frontiers */

/*!
 * @}
 */

} /* namespace CoverageControl */
#endif  // CPPSRC_CORE_INCLUDE_COVERAGECONTROL_TYPEDEFS_H_
