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
 * \file voronoi.cpp
 * \brief Functions to compute Centroidal Voronoi Tessellation
 */

#include <omp.h>

#include <functional>
#include <iostream>

#include "CoverageControl/cgal/config.h"
#include "CoverageControl/cgal/utilities.h"
#include "CoverageControl/constants.h"
#include "CoverageControl/voronoi.h"

namespace CoverageControl {

void VoronoiCell::ComputeFinalCentroid() {
  if (mass_ < kEps) {
    Polygon_2 polygon;
    for (auto const &p : cell) {
      polygon.push_back(CGAL_Point2{p[0], p[1]});
    }
    auto cgal_centroid = CGAL::centroid(polygon.begin(), polygon.end());
    centroid_ = CGALtoCC(cgal_centroid);
    centroid_ += origin_shift;
  } else {
    centroid_ = centroid_ / mass_;
  }
}

void Voronoi::CellNavigator(
    VoronoiCell const &vcell,
    std::function<void(double, Point2)> evaluator_func) {
  auto const &cell = vcell.cell;
  int n = cell.size();
  int left_id = 0;
  int right_id = 0;
  for (int i = 1; i < n; ++i) {
    if (cell[i].x() < cell[left_id].x()) {
      left_id = i;
    }
    if (cell[i].x() > cell[right_id].x()) {
      right_id = i;
    }
  }

  int min_i = std::round(cell[left_id].x() / resolution_);
  min_i = min_i < 0 ? 0 : min_i;

  int max_i = std::round(cell[right_id].x() / resolution_);
  max_i = max_i > map_size_.x() ? map_size_.x() : max_i;

  auto next_id = [=](int const id) { return (id + 1) % n; };
  auto prev_id = [=](int const id) { return id == 0 ? (n - 1) : (id - 1); };

  auto cc_pt_id = next_id(left_id);  // Counter-clockwise pointer
  auto c_pt_id = prev_id(left_id);   // Clockwise pointer

  for (int i = min_i; i < max_i; ++i) {
    double x = i * resolution_ + resolution_ / 2.;
    while (true) {
      if (cc_pt_id == left_id) {
        break;
      }
      if (cell[cc_pt_id].x() < x) {
        cc_pt_id = next_id(cc_pt_id);
      } else {
        break;
      }
    }
    while (true) {
      if (c_pt_id == left_id) {
        break;
      }
      if (cell[c_pt_id].x() < x) {
        c_pt_id = prev_id(c_pt_id);
      } else {
        break;
      }
    }

    if (cell[cc_pt_id].x() < x) {
      break;
    }
    if (cell[c_pt_id].x() < x) {
      break;
    }

    auto prev_pt = cell[prev_id(cc_pt_id)];
    auto cc_pt = cell[cc_pt_id];
    auto x1 = prev_pt.x();
    auto y1 = prev_pt.y();
    auto x2 = cc_pt.x();
    auto y2 = cc_pt.y();

    if ((x2 - x1) < kEps) {
      throw std::runtime_error{"Unexpected error!"};
    }
    auto y_lower = y1 + (x - x1) * (y2 - y1) / (x2 - x1);

    auto next_pt = cell[next_id(c_pt_id)];
    auto c_pt = cell[c_pt_id];
    x1 = next_pt.x();
    y1 = next_pt.y();
    x2 = c_pt.x();
    y2 = c_pt.y();

    if ((x2 - x1) < kEps) {
      throw std::runtime_error{"Unexpected error!"};
    }
    auto y_upper = y1 + (x - x1) * (y2 - y1) / (x2 - x1);

    int min_j = std::round(y_lower / resolution_);
    min_j = min_j < 0 ? 0 : min_j;

    int max_j = std::round(y_upper / resolution_);
    max_j = max_j > map_size_.y() ? map_size_.y() : max_j;

    for (int j = min_j; j < max_j; ++j) {
      double y = j * resolution_ + resolution_ / 2.;
      Point2 curr_pt(x, y);
      auto map_val = map_->operator()(i, j);
      evaluator_func(map_val, curr_pt);
    }
  }
}

void Voronoi::ComputeMassCentroid(VoronoiCell &vcell) {
  vcell.SetZero();
  if (compute_single_ == true) {
    vcell.origin_shift = origin_shift_;
  }

  auto fp = std::bind(&VoronoiCell::MassCentroidFunctional, &vcell,
                      std::placeholders::_1, std::placeholders::_2);
  CellNavigator(vcell, fp);
  vcell.ComputeFinalCentroid();
  /* auto fp1 = std::bind(&VoronoiCell::GoalObjFunctional, &vcell,
   * std::placeholders::_1, std::placeholders::_2); */
  /* CellNavigator(vcell, fp1); */
}

void Voronoi::ComputeVoronoiCells() {
  if (num_sites_ == 1) {
    VoronoiCell vcell;
    vcell.site = sites_[0];
    vcell.cell = PointVector{Point2{0, 0}, Point2{map_size_.x(), 0},
                             Point2{map_size_.x(), map_size_.y()},
                             Point2{0, map_size_.y()}};
    ComputeMassCentroid(vcell);
    if (compute_single_ == true) {
      voronoi_cell_ = vcell;
    } else {
      voronoi_cells_[0] = vcell;
    }
    return;
  }

  /* std::cout << "d2 begin" << std::endl; */
  Delaunay_triangulation_2 dt2;
  std::vector<CGAL_Point2> CGAL_sites;
  CGAL_sites.reserve(num_sites_);
  /* std::cout << "Number of sites: " << sites_.size() << std::endl; */
  for (auto const &pt : sites_) {
    CGAL_sites.push_back(CGAL_Point2(pt.x(), pt.y()));
  }
  dt2.insert(CGAL_sites.begin(), CGAL_sites.end());

  CGAL_DelaunayHelper vor;
  dt2.draw_dual(vor);
  /* std::cout << "d2 end" << std::endl; */

  /* std::cout << "map size" << std::endl; */
  /* std::cout << map_size_.x() << " " << map_size_.y() << std::endl; */
  vor.segments_.push_back(
      Segment_2(CGAL_Point2(0, 0), CGAL_Point2(map_size_.x(), 0)));
  vor.segments_.push_back(Segment_2(CGAL_Point2(map_size_.x(), 0),
                                    CGAL_Point2(map_size_.x(), map_size_.y())));
  vor.segments_.push_back(Segment_2(CGAL_Point2(map_size_.x(), map_size_.y()),
                                    CGAL_Point2(0, map_size_.y())));
  vor.segments_.push_back(
      Segment_2(CGAL_Point2(0, map_size_.y()), CGAL_Point2(0, 0)));

  Arrangement_2 arr;
  for (auto const &seg : vor.segments_) {
    /* std::cout << seg << std::endl; */
    if (seg.is_degenerate()) {
      continue;
    }
    CGAL::insert(arr, seg);
  }
  /* std::cout << "segments pushed" << std::endl; */

  CGAL::insert(arr, vor.rays_.begin(), vor.rays_.end());
  /* for (auto const &ray : vor.rays_) { */
  /*   if (ray.is_degenerate()) { */
  /*     continue; */
  /*   } */
  /*   CGAL::insert(arr, ray); */
  /* } */
  /* std::cout << "rays inserted" << std::endl; */
  CGAL::insert(arr, vor.lines_.begin(), vor.lines_.end());
  /* for (auto const &line : vor.lines_) { */
  /*   if (line.is_degenerate()) { */
  /*     continue; */
  /*   } */
  /*   CGAL::insert(arr, line); */
  /* } */
  /* std::cout << "lines inserted" << std::endl; */
  /* CGAL::insert(arr, vor.segments_.begin(), vor.segments_.end()); */
  /* std::cout << "arr end" << std::endl; */
  // CGAL_pl cgal_pl(arr);
  /* std::cout << "cgal_pl end" << std::endl; */

  std::vector<CGAL_Query_result> query_results_vor, query_results_vor_sorted;
  if (compute_single_ == true) {
    auto pt = CGAL_sites[robot_id_];
    std::list<CGAL_Point2> temp_site; temp_site.push_back(pt);
    CGAL::locate(arr, temp_site.begin(), temp_site.end(), std::back_inserter(query_results_vor));
    Polygon_2 polygon;
    const Arrangement_2::Face_const_handle* f;
    if ((f = std::get_if<Arrangement_2::Face_const_handle>(&query_results_vor[0].second))) {
      if((*f)->is_unbounded()) {
        throw std::runtime_error{"inside the unbounded face."};
      }
    } else {
      throw std::runtime_error{"Invalid object."};
    }
    CGAL_CCBTraversal<Arrangement_2>((*f)->outer_ccb(), polygon);
    if (not polygon.is_counterclockwise_oriented()) {
      polygon.reverse_orientation();
    }
    PointVector poly_points;
    for (auto const &p : (polygon)) {
      poly_points.push_back(CGALtoCC(p));
    }
    voronoi_cell_.site = CGALtoCC(pt);
    voronoi_cell_.cell = poly_points;

    ComputeMassCentroid(voronoi_cell_);
    return;
  }

  /* std::list <Polygon_2> polygon_list; */
  /* CGAL_GeneratePolygons(arr, polygon_list); */

  /* PrunePolygons(polygon_list, map_size_); */
  // Create voronoi_cells_ such that the correct cell is assigned to the robot
  /* std::cout << "Before parallel for" << std::endl; */
  CGAL::locate(arr, CGAL_sites.begin(), CGAL_sites.end(), std::back_inserter(query_results_vor));
  /* Results (point, object) are in xy-lexicographic order */
  /* Need to sort the results to match the order of the sites */
  for (int i = 0; i < num_sites_; ++i) {

    auto it = std::find_if(query_results_vor.begin(), query_results_vor.end(),
                           [i, CGAL_sites](CGAL_Query_result const &qr) {
                             return qr.first == CGAL_sites[i];
                           });
    if (it == query_results_vor.end()) {
      throw std::runtime_error{"Could not find a query result"};
    }
    query_results_vor_sorted.push_back(*it);
  }

#pragma omp parallel for num_threads(num_sites_)
  for (int iSite = 0; iSite < num_sites_; ++iSite) {
    const Arrangement_2::Face_const_handle* f;
    if ((f = std::get_if<Arrangement_2::Face_const_handle>(&query_results_vor_sorted[iSite].second))) {
      if((*f)->is_unbounded()) {
        throw std::runtime_error{"inside the unbounded face."};
      }
    } else {
      throw std::runtime_error{"Invalid object."};
    }
    /* CGAL_CCBTraversal<Arrangement_2> ((*f)->outer_ccb(), polygon); */
    Polygon_2 polygon;
    typename Arrangement_2::Ccb_halfedge_const_circulator circ =
        (*f)->outer_ccb();
    typename Arrangement_2::Ccb_halfedge_const_circulator curr = circ;
    auto curr_pt = curr->source()->point();
    do {
      auto he = curr;
      curr_pt = he->target()->point();
      polygon.push_back(curr_pt);
      ++curr;
    } while (curr != circ);

    if (polygon.size() == 0) {
      throw std::runtime_error{"Zero size polygon"};
    }
    /* if (not polygon.is_counterclockwise_oriented()) { */
    /*   polygon.reverse_orientation(); */
    /* } */
    VoronoiCell vcell;
    vcell.site = CGALtoCC(CGAL_sites[iSite]);
    vcell.cell.reserve(polygon.size());
    for (auto const &p : polygon) {
      vcell.cell.push_back(CGALtoCC(p));
    }
    ComputeMassCentroid(vcell);
    voronoi_cells_[iSite] = vcell;
  }
}

} /* namespace CoverageControl */
