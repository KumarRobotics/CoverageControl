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
 * \file clairvoyant_cvt.h
 * \brief Clairvoyant CVT algorithm
 */

#ifndef CPPSRC_CORE_INCLUDE_COVERAGECONTROL_ALGORITHMS_CLAIRVOYANT_CVT_H_
#define CPPSRC_CORE_INCLUDE_COVERAGECONTROL_ALGORITHMS_CLAIRVOYANT_CVT_H_

#include <omp.h>

#include <algorithm>
#include <vector>

#include "CoverageControl/algorithms/abstract_controller.h"
#include "CoverageControl/coverage_system.h"
#include "CoverageControl/parameters.h"
#include "CoverageControl/typedefs.h"

namespace CoverageControl {

/*!
 * \addtogroup cpp_api
 * @{
 * \class ClairvoyantCVT
 * @}
 * Clairvoyant CVT algorithm
 * The algorithm has knowledge of the entire map in a centralized manner.
 * It uses the CVT to compute centroids of the Voronoi cells and assigns the
 * robots to the centroids. The algorithm is online---it takes localized
 * actions based on the current robot positions.
 */
class ClairvoyantCVT : public AbstractController {
 private:
  Parameters const params_;
  size_t num_robots_ = 0;
  CoverageSystem &env_;
  Voronoi voronoi_;
  PointVector robot_global_positions_;
  PointVector goals_, actions_;
  std::vector<double> voronoi_mass_;

  bool is_converged_ = false;

 public:
  ClairvoyantCVT(Parameters const &params, CoverageSystem &env)
      : ClairvoyantCVT(params, params.pNumRobots, env) {}
  ClairvoyantCVT(Parameters const &params, size_t const &num_robots,
                 CoverageSystem &env)
      : params_{params}, num_robots_{num_robots}, env_{env} {
    robot_global_positions_ = env_.GetRobotPositions();
    actions_.resize(num_robots_);
    goals_ = robot_global_positions_;
    voronoi_mass_.resize(num_robots_, 0);
    voronoi_ = Voronoi(robot_global_positions_, env_.GetWorldMap(),
                       Point2(params_.pWorldMapSize, params_.pWorldMapSize),
                       params_.pResolution);
    ComputeGoals();
  }

  PointVector GetActions() { return actions_; }

  auto GetGoals() { return goals_; }

  auto &GetVoronoi() { return voronoi_; }

  void ComputeGoals() {
    voronoi_ = Voronoi(robot_global_positions_, env_.GetWorldMap(),
                       Point2(params_.pWorldMapSize, params_.pWorldMapSize),
                       params_.pResolution);
    auto voronoi_cells = voronoi_.GetVoronoiCells();
    for (size_t iRobot = 0; iRobot < num_robots_; ++iRobot) {
      goals_[iRobot] = voronoi_cells[iRobot].centroid();
      voronoi_mass_[iRobot] = voronoi_cells[iRobot].mass();
    }
  }

  int ComputeActions() {
    is_converged_ = true;
    robot_global_positions_ = env_.GetRobotPositions();
    ComputeGoals();
    auto voronoi_cells = voronoi_.GetVoronoiCells();
    for (size_t iRobot = 0; iRobot < num_robots_; ++iRobot) {
      actions_[iRobot] = Point2(0, 0);
      Point2 diff = goals_[iRobot] - robot_global_positions_[iRobot];
      double dist = diff.norm();
      /* if (dist < 0.1 * params_.pResolution) { */
      /*   continue; */
      /* } */
      if (dist < kEps) {
        continue;
      }
      if (env_.CheckOscillation(iRobot)) {
        continue;
      }
      double speed = dist / params_.pTimeStep;
      /* double speed = 2 * dist * voronoi_mass_[iRobot]; */
      speed = std::min(params_.pMaxRobotSpeed, speed);
      Point2 direction(diff);
      direction.normalize();
      actions_[iRobot] = speed * direction;
      is_converged_ = false;
    }
    return 0;
  }

  bool IsConverged() const { return is_converged_; }
};

}  // namespace CoverageControl
#endif  // CPPSRC_CORE_INCLUDE_COVERAGECONTROL_ALGORITHMS_CLAIRVOYANT_CVT_H_
