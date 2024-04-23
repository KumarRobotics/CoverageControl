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
 * \file polygon_utils.h
 * \brief Provides utilities for polygon manipulation using CGAL.
 */

#ifndef CPPSRC_CORE_INCLUDE_COVERAGECONTROL_CGAL_POLYGON_UTILS_H_
#define CPPSRC_CORE_INCLUDE_COVERAGECONTROL_CGAL_POLYGON_UTILS_H_

#include <vector>

#include "CoverageControl/typedefs.h"

namespace CoverageControl {

/*! \brief Partition a polygon into y-monotone polygons
 *
 * @param[in] polygon The input polygon
 * @param[out] y_monotone_polygons The output y-monotone polygons
 */
void PolygonYMonotonePartition(PointVector const &polygon,
                               std::vector<PointVector> &y_monotone_polygons);

/*! \brief Generate random polygons
 *
 * @param[in] num_polygons The number of polygons to generate
 * @param[in] max_vertices The maximum number of vertices in each polygon
 * @param[in] half_width The half-width of the polygon
 * @param[in] world_size The size of the world
 * @param[out] polygons The output polygons
 */

void GenerateRandomPolygons(int const num_polygons, int const max_vertices,
                            double const half_width, double const world_size,
                            std::vector<PointVector> &polygons);

} /* namespace CoverageControl */

#endif  // CPPSRC_CORE_INCLUDE_COVERAGECONTROL_CGAL_POLYGON_UTILS_H_
