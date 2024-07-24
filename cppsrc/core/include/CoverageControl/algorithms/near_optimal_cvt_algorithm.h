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
 * \file near_optimal_cvt_algorithm.h
 * \brief Near Optimal Centroidal Voronoi Tessellation (CVT) algorithm
 * Near-optimal Centroidal Voronoi Tessellation (CVT) algorithms.
 * The algorithm has knowledge of the entire map in a centralized manner.
 * It spawns random sites and iteratively moves them to the centroid of the
 * Voronoi cell, until convergence. Hungarian algorithm is used to assign the
 * robots to the sites. Out of the multiple tries, the best Voronoi is selected
 * based on the objective function.
 */

#ifndef CPPSRC_CORE_INCLUDE_COVERAGECONTROL_ALGORITHMS_NEAR_OPTIMAL_CVT_ALGORITHM_H_
#define CPPSRC_CORE_INCLUDE_COVERAGECONTROL_ALGORITHMS_NEAR_OPTIMAL_CVT_ALGORITHM_H_

#define EIGEN_NO_CUDA  // Don't use eigen's cuda facility
#include <omp.h>
#include <time.h>

#include <Eigen/Dense>  // Eigen is used for maps
#include <random>
#include <vector>

#include "CoverageControl/extern/lsap/Hungarian.h"
#include "CoverageControl/typedefs.h"
#include "CoverageControl/voronoi.h"

namespace CoverageControl {

/*!
 * \addtogroup cpp_api
 * @{
 */

inline auto NearOptimalCVTAlgorithm(int const num_tries,
                                    int const max_iterations,
                                    int const num_sites, MapType const &map,
                                    int const map_size, double const res) {
  std::random_device
      rd_;  // Will be used to obtain a seed for the random number engine
  std::mt19937 gen_;
  std::srand(time(NULL));
  gen_ = std::mt19937(
      rd_());  // Standard mersenne_twister_engine seeded with rd_()
  std::vector<Voronoi> all_voronoi_cells;
  /* all_voronoi_cells.resize(num_tries, std::vector<VoronoiCell>(num_sites));
   */
  all_voronoi_cells.resize(num_tries);
  std::vector<double> obj_values;
  obj_values.resize(num_tries, 0);
  std::uniform_real_distribution<> distrib_pts(0.001, map_size * res - 0.001);

#pragma omp parallel for
  for (int iter = 0; iter < num_tries; ++iter) {
    PointVector sites;
    sites.resize(num_sites);
    for (int iSite = 0; iSite < num_sites; ++iSite) {
      sites[iSite] = Point2(distrib_pts(gen_), distrib_pts(gen_));
    }
    bool cont_flag = true;
    /* std::cout << "voronoi start" << std::endl; */
    Voronoi voronoi(sites, map, Point2(map_size, map_size), res);
    /* std::cout << "voronoi end" << std::endl; */
    auto voronoi_cells = voronoi.GetVoronoiCells();
    int iSteps = 0;
    for (iSteps = 0; iSteps < max_iterations and cont_flag == true; ++iSteps) {
      cont_flag = false;
      voronoi_cells = voronoi.GetVoronoiCells();
      for (int iSite = 0; iSite < num_sites; ++iSite) {
        Point2 diff =
            voronoi_cells[iSite].centroid() - voronoi_cells[iSite].site;
        if (diff.norm() < res) {
          continue;
        }
        cont_flag = true;
        sites[iSite] = voronoi_cells[iSite].centroid();
      }
      voronoi.UpdateSites(sites);
    }
    /* std::cout << "No. of voronoi steps: " << iSteps << std::endl; */
    all_voronoi_cells[iter] = voronoi;
    obj_values[iter] = voronoi.GetSumIDFGoalDistSqr();
  }
  int best_vornoi_idx = 0;
  double min = obj_values[0];
  for (int iter = 1; iter < num_tries; ++iter) {
    if (obj_values[iter] < min) {
      min = obj_values[iter];
      best_vornoi_idx = iter;
    }
  }
  return all_voronoi_cells[best_vornoi_idx];
}

inline auto NearOptimalCVTAlgorithm(int const num_tries,
                                    int const max_iterations,
                                    int const num_sites, MapType const &map,
                                    int const map_size, double const res,
                                    PointVector const &positions,
                                    Voronoi &voronoi) {
  voronoi = NearOptimalCVTAlgorithm(num_tries, max_iterations, num_sites, map,
                                    map_size, res);
  auto voronoi_cells = voronoi.GetVoronoiCells();
  std::vector<std::vector<double>> cost_matrix;
  cost_matrix.resize(num_sites, std::vector<double>(num_sites));
#pragma omp parallel for num_threads(num_sites)
  for (int iRobot = 0; iRobot < num_sites; ++iRobot) {
    for (int jCentroid = 0; jCentroid < num_sites; ++jCentroid) {
      cost_matrix[iRobot][jCentroid] =
          (positions[iRobot] - voronoi_cells[jCentroid].centroid()).norm();
    }
  }
  HungarianAlgorithm HungAlgo;
  std::vector<int> assignment;
  HungAlgo.Solve(cost_matrix, assignment);

  PointVector goals;
  goals.resize(num_sites);
  for (int iRobot = 0; iRobot < num_sites; ++iRobot) {
    goals[iRobot] = voronoi_cells[assignment[iRobot]].centroid();
  }

  return goals;
}

inline auto NearOptimalCVTAlgorithm(int const num_tries,
                                    int const max_iterations,
                                    int const num_sites, MapType const &map,
                                    int const map_size, double const res,
                                    PointVector const &positions) {
  Voronoi voronoi;
  return NearOptimalCVTAlgorithm(num_tries, max_iterations, num_sites, map,
                                 map_size, res, positions, voronoi);
}

/*!
 * @}
 */
} /* namespace CoverageControl */
#endif  // CPPSRC_CORE_INCLUDE_COVERAGECONTROL_ALGORITHMS_NEAR_OPTIMAL_CVT_ALGORITHM_H_
