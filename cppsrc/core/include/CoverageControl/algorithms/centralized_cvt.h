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
 * \file centralized_cvt.h
 * \brief Contains the class CentralizedCVT
 */

#ifndef CPPSRC_CORE_INCLUDE_COVERAGECONTROL_ALGORITHMS_CENTRALIZED_CVT_H_
#define CPPSRC_CORE_INCLUDE_COVERAGECONTROL_ALGORITHMS_CENTRALIZED_CVT_H_

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
 * \class CentralizedCVT
 * @}
 * The coverage control algorithm uses centralized Centroidal Voronoi
 *Tessellation (CVT) or Lloyd's algorithm on accumulated local map of individual
 *robots, i.e., a robot has knowledge only about the regions it has visited.
 * Communication radius is not considered.
 * The algorithm is online---it takes localized actions based on the current
 *robot positions.
 **/
class CentralizedCVT : public AbstractController {
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
  CentralizedCVT(Parameters const &params, CoverageSystem &env)
      : CentralizedCVT(params, params.pNumRobots, env) {}
  CentralizedCVT(Parameters const &params, size_t const &num_robots,
                 CoverageSystem &env)
      : params_{params}, num_robots_{num_robots}, env_{env} {
    robot_global_positions_ = env_.GetRobotPositions();
    actions_.resize(num_robots_);
    goals_ = robot_global_positions_;
    voronoi_mass_.resize(num_robots_, 0);
    voronoi_ = Voronoi(robot_global_positions_, env_.GetSystemExploredIDFMap(),
                       Point2(params_.pWorldMapSize, params_.pWorldMapSize),
                       params_.pResolution);
    ComputeGoals();
  }

  PointVector GetActions() { return actions_; }

  auto GetGoals() { return goals_; }

  auto &GetVoronoi() { return voronoi_; }

  void ComputeGoals() {
    voronoi_ = Voronoi(robot_global_positions_, env_.GetSystemExploredIDFMap(),
                       Point2(params_.pWorldMapSize, params_.pWorldMapSize),
                       params_.pResolution);
    auto voronoi_cells = voronoi_.GetVoronoiCells();
    for (size_t iRobot = 0; iRobot < num_robots_; ++iRobot) {
      goals_[iRobot] = voronoi_cells[iRobot].centroid();
      voronoi_mass_[iRobot] = voronoi_cells[iRobot].mass();
    }
  }

  bool IsConverged() const { return is_converged_; }

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
};

} /* namespace CoverageControl */
#endif  // CPPSRC_CORE_INCLUDE_COVERAGECONTROL_ALGORITHMS_CENTRALIZED_CVT_H_
