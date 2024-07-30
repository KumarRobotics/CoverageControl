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
 * \file voronoi.h
 * \brief Class for computing Voronoi cells
 */

#ifndef CPPSRC_CORE_INCLUDE_COVERAGECONTROL_VORONOI_H_
#define CPPSRC_CORE_INCLUDE_COVERAGECONTROL_VORONOI_H_

#include <cmath>
#include <memory>
#include <vector>

#include "CoverageControl/typedefs.h"

namespace CoverageControl {

/*!
 * \addtogroup cpp_api
 * @{
 * \class VoronoiCell
 * @}
 * \brief Struct for Voronoi cell
 *
 * This struct contains the site of the Voronoi cell and the points that make up
 * the cell. Functions are provided to compute the mass and centroid of the
 * cell.
 *
 */
struct VoronoiCell {
  Point2 site;
  PointVector cell;
  Point2 origin_shift = Point2(0, 0);

 private:
  double mass_ = 0;
  Point2 centroid_;
  double sum_idf_site_dist_sqr_ = 0;
  double sum_idf_goal_dist_sqr_ = 0;
  double sum_idf_site_dist_ = 0;
  double sum_idf_goal_dist_ = 0;

 public:
  Point2 centroid() const { return centroid_; }
  double mass() const { return mass_; }
  double sum_idf_site_dist() const { return sum_idf_site_dist_; }
  double sum_idf_site_dist_sqr() const { return sum_idf_site_dist_sqr_; }
  double sum_idf_goal_dist() const { return sum_idf_goal_dist_; }
  double sum_idf_goal_dist_sqr() const { return sum_idf_goal_dist_sqr_; }

  std::vector<double> GetFeatureVector() const {
    return std::vector<double>{
        centroid_.x(),          centroid_.y(),          mass_,
        sum_idf_site_dist_sqr_, sum_idf_goal_dist_sqr_, sum_idf_site_dist_,
        sum_idf_goal_dist_};
  }

  void SetZero() {
    mass_ = 0;
    centroid_ = Point2{0, 0};
    origin_shift = Point2{0, 0};
    sum_idf_site_dist_ = 0;
    sum_idf_site_dist_sqr_ = 0;
    sum_idf_goal_dist_ = 0;
    sum_idf_goal_dist_sqr_ = 0;
  }
  void MassCentroidFunctional(double const &map_val, Point2 pt) {
    pt = pt + origin_shift;
    mass_ += map_val;
    centroid_ += pt * map_val;
    sum_idf_site_dist_ += (pt - site).norm() * map_val;
    sum_idf_site_dist_sqr_ += (pt - site).squaredNorm() * map_val;
  }
  void GoalObjFunctional(double const &map_val, Point2 pt) {
    pt = pt + origin_shift;
    sum_idf_goal_dist_sqr_ += (pt - centroid_).squaredNorm() * map_val;
    sum_idf_goal_dist_ += (pt - centroid_).norm() * map_val;
  }

  void ComputeFinalCentroid();
};

/*!
 * \addtogroup cpp_api
 * @{
 * \class Voronoi
 * @}
 * \brief Class for computing Voronoi cells
 *
 * This class contains the sites of the Voronoi cells and the map. It computes
 * the Voronoi cells and stores the mass and centroid of each cell.
 *
 */
class Voronoi {
 private:
  PointVector sites_;
  std::shared_ptr<const MapType> map_ = nullptr;
  Point2 map_size_;
  double resolution_ = 0;
  Point2 origin_shift_;

  // compute_single_ is used to determine if the voronoi mass and centroid needs
  // to be computed for only a single site, given by robot_id_. This is useful
  // for distributed voronoi computation The result is stored in voronoi_cell_
  // (as opposed to voronoi_cells_ when compute_single_ = false)
  bool compute_single_ = false;
  int robot_id_ = 0;
  VoronoiCell voronoi_cell_;

  int num_sites_;
  std::vector<VoronoiCell> voronoi_cells_;
  void ComputeMassCentroid(VoronoiCell &);
  void ComputeMassCentroid2(VoronoiCell &);
  void MassCentroidFunctional(VoronoiCell &vcell, double const &map_val,
                              Point2 const &pt);
  void CellNavigator(VoronoiCell const &, std::function<void(double, Point2)>);

  /* std::vector <Edge> voronoi_edges_; */
 public:
  Voronoi() {}
  Voronoi(PointVector const &sites, MapType const &map, Point2 map_size,
          double const &resolution, bool const compute_single = false,
          int const robot_id = 0)
      : sites_{sites},
        map_size_{map_size},
        resolution_{resolution},
        compute_single_{compute_single},
        robot_id_{robot_id} {
    /* std::cout << "map size" << std::endl; */
    /* std::cout << map_size_.x() << " " << map_size_.y() << std::endl; */
    map_ = std::make_shared<const MapType>(map);
    num_sites_ = sites_.size();
    /* double shortest_dist = std::numeric_limits<double>::max(); */
    /* for (int i = 0; i < num_sites_; ++i) { */
    /*   auto const &site = sites[i]; */
    /*   for (int j = i + 1; j < num_sites_; ++j) { */
    /*     auto const &site2 = sites[j]; */
    /*     double dist = (site - site2).norm(); */
    /*     if (dist < shortest_dist) { */
    /*       shortest_dist = dist; */
    /*     } */
    /*   } */
    /* } */
    /* std::cout << "Shortest dist between sites: " << shortest_dist <<
     * std::endl; */
    if (compute_single_ == false) {
      voronoi_cells_.resize(num_sites_);
    } else {
      origin_shift_ = -sites[0];
    }

    ComputeVoronoiCells();
  }

  // Update the cites and recompute the voronoi
  void UpdateSites(PointVector const &sites) {
    sites_ = sites;
    num_sites_ = sites_.size();
    ComputeVoronoiCells();
  }

  void ComputeVoronoiCells();
  auto GetVoronoiCells() const { return voronoi_cells_; }
  auto GetVoronoiCell() { return voronoi_cell_; }

  double GetSumIDFSiteDistSqr() {
    double obj = 0;
    for (auto const &cell : voronoi_cells_) {
      obj = obj + cell.sum_idf_site_dist_sqr();
    }
    return obj;
  }
  double GetSumIDFGoalDistSqr() {
    double obj = 0;
    for (auto const &cell : voronoi_cells_) {
      obj = obj + cell.sum_idf_goal_dist_sqr();
    }
    return obj;
  }
  /* auto GetVoronoiEdges() {return voronoi_edges_;} */
};

} /* namespace CoverageControl */
#endif  // CPPSRC_CORE_INCLUDE_COVERAGECONTROL_VORONOI_H_
