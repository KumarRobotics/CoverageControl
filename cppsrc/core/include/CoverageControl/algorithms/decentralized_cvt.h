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
 * \file decentralized_cvt.h
 * \brief Contains the DecentralizedCVT class.
 */

#ifndef CPPSRC_CORE_INCLUDE_COVERAGECONTROL_ALGORITHMS_DECENTRALIZED_CVT_H_
#define CPPSRC_CORE_INCLUDE_COVERAGECONTROL_ALGORITHMS_DECENTRALIZED_CVT_H_

#include <omp.h>

#include <algorithm>
#include <iostream>
#include <vector>

#include "CoverageControl/algorithms/abstract_controller.h"
#include "CoverageControl/coverage_system.h"
#include "CoverageControl/map_utils.h"
#include "CoverageControl/parameters.h"
#include "CoverageControl/typedefs.h"

namespace CoverageControl {

/*!
 * \addtogroup cpp_api
 * @{
 * \class DecentralizedCVT
 * @}
 * This class implements the decentralized CVT algorithm.
 * The algorithm uses the local map of each robot and the positions of other
 * robots within its communication range to compute the voronoi. The algorithm
 * is online---it takes localized actions based on the current robot
 * positions.
 */
class DecentralizedCVT : public AbstractController {
 private:
  Parameters const params_;
  size_t num_robots_ = 0;
  CoverageSystem &env_;
  PointVector robot_global_positions_;
  PointVector goals_, actions_;
  std::vector<double> voronoi_mass_;

  bool is_converged_ = false;

 public:
  DecentralizedCVT(Parameters const &params, CoverageSystem &env)
      : DecentralizedCVT(params, params.pNumRobots, env) {}
  DecentralizedCVT(Parameters const &params, size_t const &num_robots,
                   CoverageSystem &env)
      : params_{params}, num_robots_{num_robots}, env_{env} {
    robot_global_positions_ = env_.GetRobotPositions();
    actions_.resize(num_robots_);
    goals_ = robot_global_positions_;
    voronoi_mass_.resize(num_robots_, 0);
    ComputeGoals();
  }

  PointVector GetActions() { return actions_; }

  PointVector GetGoals() { return goals_; }

  void ComputeGoals() {
#pragma omp parallel for num_threads(num_robots_)
    for (size_t iRobot = 0; iRobot < num_robots_; ++iRobot) {
      Point2 const &pos = robot_global_positions_[iRobot];
      MapUtils::MapBounds index, offset;
      MapUtils::ComputeOffsets(params_.pResolution, pos, params_.pLocalMapSize,
                               params_.pWorldMapSize, index, offset);
      MapType robot_map = env_.GetRobotMap(iRobot);
      MapType robot_local_map = robot_map.block(index.left + offset.left,
                                             index.bottom + offset.bottom,
                                             offset.width, offset.height);
      Point2 map_translation(
          (index.left + offset.left) * params_.pResolution,
          (index.bottom + offset.bottom) * params_.pResolution);

      PointVector robot_neighbors_pos;
      for (size_t jRobot = 0; jRobot < num_robots_; ++jRobot) {
        if (iRobot == jRobot) {
          continue;
        }
        Point2 relative_pos =
            robot_global_positions_[jRobot] - robot_global_positions_[iRobot];
        if (relative_pos.norm() < params_.pCommunicationRange) {
          robot_neighbors_pos.push_back(robot_global_positions_[jRobot]);
        }
      }
      PointVector robot_positions(robot_neighbors_pos.size() + 1);

      robot_positions[0] = robot_global_positions_[iRobot] - map_translation;
      /* std::cout << "Voronoi: " << robot_positions[0][0] << " " <<
       * robot_positions[0][1] << std::endl; */
      int count = 1;
      for (Point2 const &pos : robot_neighbors_pos) {
        robot_positions[count] = pos - map_translation;
        ++count;
      }
      Point2 map_size(offset.width, offset.height);
      Voronoi voronoi(robot_positions, robot_local_map, map_size,
                      params_.pResolution, true, 0);
      auto vcell = voronoi.GetVoronoiCell();
      voronoi_mass_[iRobot] = vcell.mass();
      goals_[iRobot] = vcell.centroid() + robot_positions[0] + map_translation;
      if (goals_[iRobot][0] < 0 or goals_[iRobot][0] > params_.pWorldMapSize or
          goals_[iRobot][1] < 0 or goals_[iRobot][1] > params_.pWorldMapSize) {
        std::cout << "Goal out of bounds: " << goals_[iRobot][0] << " "
                  << goals_[iRobot][1] << std::endl;
        std::cout << robot_global_positions_[iRobot][0] << " "
                  << robot_global_positions_[iRobot][1] << std::endl;
        std::cout << vcell.centroid()[0] << " " << vcell.centroid()[1]
                  << std::endl;
        std::cout << map_translation[0] << " " << map_translation[1]
                  << std::endl;
        std::cout << robot_local_map.sum() << std::endl;
        throw std::runtime_error(
            "Goal out of bounds: this should not have happened for convex "
            "environments");
      }
    }
  }

  int ComputeActions() {
    is_converged_ = true;
    robot_global_positions_ = env_.GetRobotPositions();
    ComputeGoals();
    for (size_t iRobot = 0; iRobot < num_robots_; ++iRobot) {
      actions_[iRobot] = Point2(0, 0);
      Point2 diff = goals_[iRobot] - robot_global_positions_[iRobot];
      double dist = diff.norm();
      /* std::cout << "Robot " << iRobot << " goal: " << goals_[iRobot][0] <<
       * " " << goals_[iRobot][1] << " " << dist << std::endl; */
      if (dist < kEps) {
        continue;
      }
      if (env_.CheckOscillation(iRobot)) {
        continue;
      }
      double speed = dist / params_.pTimeStep;
      /* double speed = 2 * dist * voronoi_mass_[iRobot]; */
      /* if (dist > 0 and voronoi_mass_[iRobot] == 0) { */
      /*   speed = dist / params_.pTimeStep; */
      /* } */
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
#endif  // CPPSRC_CORE_INCLUDE_COVERAGECONTROL_ALGORITHMS_DECENTRALIZED_CVT_H_
