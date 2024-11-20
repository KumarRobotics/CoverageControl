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
 * \file polygon_utils.cpp
 * \brief Helper functions for polygon operations using CGAL
 */

#include <list>
#include <vector>

#include "CoverageControl/cgal/config.h"
#include "CoverageControl/cgal/polygon_utils.h"
#include "CoverageControl/cgal/utilities.h"
#include "CoverageControl/constants.h"

namespace CoverageControl {
void PolygonYMonotonePartition(PointVector const &poly,
                               std::vector<PointVector> &new_polys) {
  // Transform general polygon to CGAL //
  Partition_traits_2::Polygon_2 cgal_poly;
  for (auto const &pt : poly) {
    cgal_poly.push_back(Partition_traits_2::Point_2(pt.x(), pt.y()));
  }

  /* std::cout << "Is simple: " << cgal_poly.is_simple() << std::endl; */
  /* std::cout << "Is orientation: " << cgal_poly.orientation() << std::endl; */
  if (cgal_poly.orientation() == CGAL::CLOCKWISE) {
    cgal_poly.reverse_orientation();
  }
  // Obtain partition //
  std::list<Partition_traits_2::Polygon_2> partition_polys;
  CGAL::y_monotone_partition_2(cgal_poly.vertices_begin(), cgal_poly.vertices_end(),
                               std::back_inserter(partition_polys));

  // Transform to coveragecontrol data type

  new_polys.reserve(new_polys.size() + partition_polys.size());
  for (auto const &p : partition_polys) {
    PointVector new_p;
    new_p.reserve(p.size());
    std::transform(p.vertices_begin(), p.vertices_end(),
                   std::back_inserter(new_p),
                   [](CGAL_Point2 const &pt) { return CGALtoCC(pt); });
    new_polys.push_back(new_p);
  }
}

void GenerateRandomPolygons(int const num_polygons, int const max_vertices,
                            double const half_width, double const world_size,
                            std::vector<PointVector> &polygons) {
  polygons.clear();
  polygons.reserve(num_polygons);
  CGAL::Random rand;
  for (int i = 0; i < num_polygons; ++i) {
    int num_vertices = rand.get_int(4, max_vertices);
    PointVector poly;
    poly.reserve(num_vertices);
    std::list<CGAL_Point2> points;
    CGAL::copy_n_unique(Point_generator(half_width), num_vertices,
                        std::back_inserter(points));
    Polygon_2 poly_cgal;
    CGAL::random_polygon_2(num_vertices, std::back_inserter(poly_cgal),
                           points.begin());
    auto bbox = poly_cgal.bbox();
    CGAL_Point2 lower_left(bbox.xmin(), bbox.ymin());
    CGAL_Point2 upper_right(bbox.xmax() - bbox.xmin(),
                            bbox.ymax() - bbox.ymin());
    CGAL_Point2 polygon_base = CGAL_Point2(
        rand.get_double(kLargeEps, world_size - kLargeEps -
                                       CGAL::to_double(upper_right.x())),
        rand.get_double(kLargeEps, world_size - kLargeEps -
                                       CGAL::to_double(upper_right.y())));
    for (auto const &pt : poly_cgal) {
      Point2 new_pt = CGALtoCC(pt);
      new_pt += CGALtoCC(polygon_base);
      new_pt -= CGALtoCC(lower_left);
      poly.push_back(new_pt);
    }
    polygons.push_back(poly);
  }
}

} /* namespace CoverageControl */
