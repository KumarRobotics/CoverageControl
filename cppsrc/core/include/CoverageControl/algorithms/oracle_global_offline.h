/*
 * This file is part of the CoverageControl library
 *
 * Author: Saurav Agarwal
 * Contact: sauravag@seas.upenn.edu, agr.saurav1@gmail.com
 * Repository: https://github.com/KumarRobotics/CoverageControl
 *
 * The CoverageControl library is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
 *
 * The CoverageControl library is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with CoverageControl library. If not, see <https://www.gnu.org/licenses/>.
 */

/*!
 * \file oracle_global_offline.h
 * \brief OracleGlobalOffline class
 * Global offline oracle: has complete information about the environment.
 * Use full communication between robots.
 * Tries multiple random sites for initial locations.
 * Uses Hungarian algorithm to assign robots to goal positions.
 */


#ifndef COVERAGECONTROL_ORACLE_GLOBAL_OFFLINE_H_
#define COVERAGECONTROL_ORACLE_GLOBAL_OFFLINE_H_

#include <vector>
#include <fstream>
#include <iostream>
#include <random>
#include <algorithm>
#include <set>
#include <queue>
#include <omp.h>

#include "../parameters.h"
#include "../typedefs.h"
#include "../coverage_system.h"
#include "../map_utils.h"
#include "near_optimal_cvt.h"
#include "../extern/lsap/Hungarian.h"

namespace CoverageControl {

	class OracleGlobalOffline {
		private:
			Parameters const params_;
			size_t num_robots_ = 0;
			CoverageSystem &env_;
			Voronoi voronoi_;
			PointVector robot_global_positions_;
			PointVector goals_, actions_;

			bool continue_flag_ = false;

		public:
			OracleGlobalOffline(
					Parameters const &params,
					size_t const &num_robots,
					CoverageSystem &env) :
				params_{params},
				num_robots_{num_robots},
				env_{env} {

					robot_global_positions_ = env_.GetRobotPositions();
					actions_.resize(num_robots_);
					goals_ = robot_global_positions_;
					ComputeGoals();
				}

			PointVector GetActions() { return actions_; }

			auto GetGoals() { return goals_; }

			auto &GetVoronoi() { return voronoi_; }

			void ComputeGoals() {
				goals_ = NearOptimalCVT(params_.pLloydNumTries, params_.pLloydMaxIterations, num_robots_, env_.GetWorldIDF(), params_.pWorldMapSize, params_.pResolution, robot_global_positions_, voronoi_);
			}

			bool Step() {
				continue_flag_ = false;
				robot_global_positions_ = env_.GetRobotPositions();
				for(size_t iRobot = 0; iRobot < num_robots_; ++iRobot) {
					actions_[iRobot] = Point2(0, 0);
					Point2 diff = goals_[iRobot] - robot_global_positions_[iRobot];
					double dist = diff.norm();
					if(dist < kEps) {
						continue;
					}
					double speed = dist / params_.pTimeStep;
					speed = std::min(params_.pMaxRobotSpeed, speed);
					Point2 direction(diff);
					direction.normalize();
					actions_[iRobot] = speed * direction;
					continue_flag_ = true;
				}
				return continue_flag_;
			}

	};

} /* namespace CoverageControl */
#endif /* COVERAGECONTROL_ORACLE_GLOBAL_OFFLINE_H_ */
