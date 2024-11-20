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
 * \file utilities.h
 * \brief Contains utility functions for CGAL
 */

#ifndef CPPSRC_CORE_INCLUDE_COVERAGECONTROL_CGAL_UTILITIES_H_
#define CPPSRC_CORE_INCLUDE_COVERAGECONTROL_CGAL_UTILITIES_H_

#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <omp.h>

#include <list>
#include <vector>

#include "CoverageControl/cgal/config.h"
#include "CoverageControl/typedefs.h"

namespace CoverageControl {

inline Point2 CGALtoCC(CGAL_Point2 const pt) {
  return Point2(CGAL::to_double(pt.x()), CGAL::to_double(pt.y()));
}
struct CGAL_DelaunayHelper {
  std::vector<Ray_2> rays_;
  std::vector<Line_2> lines_;
  std::vector<Segment_2> segments_;
  CGAL_DelaunayHelper() {}
  void operator<<(const Ray_2 &ray) { rays_.push_back(ray); }
  void operator<<(const Line_2 &line) { lines_.push_back(line); }
  void operator<<(const Segment_2 &seg) { segments_.push_back(seg); }
};

template <class Arrangement>
inline void CGAL_CCBTraversal(
    typename Arrangement::Ccb_halfedge_const_circulator circ,
    Polygon_2 &polygon) {
  polygon.clear();
  typename Arrangement::Ccb_halfedge_const_circulator curr = circ;
  typename Arrangement::Halfedge_const_handle he;
  auto pt = curr->source()->point();
  do {
    he = curr;
    pt = he->target()->point();
    polygon.push_back(pt);
    ++curr;
  } while (curr != circ);
}

template <class Arrangement>
inline void CGAL_GeneratePolygons(const Arrangement &arr,
                                  std::list<Polygon_2> &polygon_list) {
  /* CGAL_precondition (arr.is_valid()); */
  typename Arrangement::Face_const_iterator fit;
  for (fit = arr.faces_begin(); fit != arr.faces_end(); ++fit) {
    if (fit->is_unbounded()) {
      continue;
    } else {
      Polygon_2 polygon;
      CGAL_CCBTraversal<Arrangement>(fit->outer_ccb(), polygon);
      if (not polygon.is_counterclockwise_oriented()) {
        polygon.reverse_orientation();
      }
      polygon_list.push_back(polygon);
    }
  }
}

inline bool IsPointInPoly(CGAL_Point2 const &pt, Polygon_2 const &poly) {
  if (CGAL::bounded_side_2(poly.begin(), poly.end(), pt, K()) ==
      CGAL::ON_UNBOUNDED_SIDE) {
    return false;
  }
  return true;
}

inline void PrunePolygons(std::list<Polygon_2> &polygon_list,
                          int const &map_size) {
  Polygon_2 bbox_poly;
  bbox_poly.push_back(CGAL_Point2(0, 0));
  bbox_poly.push_back(CGAL_Point2(map_size, 0));
  bbox_poly.push_back(CGAL_Point2(map_size, map_size));
  bbox_poly.push_back(CGAL_Point2(0, map_size));
  for (auto it = polygon_list.begin(); it != polygon_list.end();) {
    if (not CGAL::do_intersect(bbox_poly, *it)) {
      it = polygon_list.erase(it);
    } else {
      ++it;
    }
  }
}

} /* namespace CoverageControl */
#endif  // CPPSRC_CORE_INCLUDE_COVERAGECONTROL_CGAL_UTILITIES_H_
