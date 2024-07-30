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
 * \file near_optimal_cvt.h
 * \brief NearOptimalCVT class
 * Global offline oracle: has complete information about the environment.
 * Use full communication between robots.
 * Tries multiple random sites for initial locations.
 * Uses Hungarian algorithm to assign robots to goal positions.
 */

#ifndef CPPSRC_CORE_INCLUDE_COVERAGECONTROL_ALGORITHMS_NEAR_OPTIMAL_CVT_H_
#define CPPSRC_CORE_INCLUDE_COVERAGECONTROL_ALGORITHMS_NEAR_OPTIMAL_CVT_H_

#include <omp.h>

#include <algorithm>
#include <vector>

#include "CoverageControl/algorithms/abstract_controller.h"
#include "CoverageControl/algorithms/near_optimal_cvt_algorithm.h"
#include "CoverageControl/coverage_system.h"
#include "CoverageControl/parameters.h"
#include "CoverageControl/typedefs.h"

namespace CoverageControl {

/*!
 * \addtogroup cpp_api
 * @{
 * \class NearOptimalCVT
 * @}
 * The algorithm has knowledge of the entire map in a centralized manner.
 * It runs several trials; for each trial, it generates random initial positions
 *for the sites. Then uses the CVT to compute optimal centroids. It performs the
 *Hungarian algorithm to assign robots to the centroids. Finally, the trial with
 *the minimum cost is selected.
 **/
class NearOptimalCVT : public AbstractController {
 private:
  Parameters const params_;
  size_t num_robots_ = 0;
  CoverageSystem &env_;
  Voronoi voronoi_;
  PointVector robot_global_positions_;
  PointVector goals_, actions_;

  bool is_converged_ = false;

 public:
  NearOptimalCVT(Parameters const &params, CoverageSystem &env)
      : NearOptimalCVT(params, params.pNumRobots, env) {}
  NearOptimalCVT(Parameters const &params, size_t const &num_robots,
                 CoverageSystem &env)
      : params_{params}, num_robots_{num_robots}, env_{env} {
    robot_global_positions_ = env_.GetRobotPositions();
    actions_.resize(num_robots_);
    goals_ = robot_global_positions_;
    ComputeGoals();
  }

  PointVector GetActions() { return actions_; }

  auto GetGoals() { return goals_; }

  auto &GetVoronoi() { return voronoi_; }

  void ComputeGoals() {
    goals_ = NearOptimalCVTAlgorithm(
        params_.pLloydNumTries, params_.pLloydMaxIterations, num_robots_,
        env_.GetWorldMap(), params_.pWorldMapSize, params_.pResolution,
        robot_global_positions_, voronoi_);
  }

  int ComputeActions() {
    is_converged_ = true;
    robot_global_positions_ = env_.GetRobotPositions();
    for (size_t iRobot = 0; iRobot < num_robots_; ++iRobot) {
      actions_[iRobot] = Point2(0, 0);
      Point2 diff = goals_[iRobot] - robot_global_positions_[iRobot];
      double dist = diff.norm();
      if (dist < kEps) {
        continue;
      }
      double speed = dist / params_.pTimeStep;
      speed = std::min(params_.pMaxRobotSpeed, speed);
      Point2 direction(diff);
      direction.normalize();
      actions_[iRobot] = speed * direction;
      is_converged_ = false;
    }
    return 0;
  }

  bool IsConverged() { return is_converged_; }
};

} /* namespace CoverageControl */
#endif  // CPPSRC_CORE_INCLUDE_COVERAGECONTROL_ALGORITHMS_NEAR_OPTIMAL_CVT_H_
