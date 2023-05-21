/**
 * The coverage control algorithm uses Lloyd's algorithm on global world IDF map, i.e., it has knowledge of the entire map.
 * Communication is not considered in this algorithm, i.e., all robots are assumed to be able to communicate with each other.
 * However, the algorithm is online---it takes localized actions based on the current robot positions.
 **/

#ifndef COVERAGECONTROL_ALGORITHMS_LLOYD_GLOBAL_ONLINE_H_
#define COVERAGECONTROL_ALGORITHMS_LLOYD_GLOBAL_ONLINE_H_

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
#include "lloyd_algorithms.h"
#include "../extern/lsap/Hungarian.h"

namespace CoverageControl {

	class LloydGlobalOnline {
		private:
			Parameters const params_;
			size_t num_robots_ = 0;
			CoverageSystem &env_;
			Voronoi voronoi_;
			PointVector robot_global_positions_;
			PointVector goals_, actions_;
			std::vector <double> voronoi_mass_;

			bool continue_flag_ = false;

		public:
			LloydGlobalOnline(
					Parameters const &params,
					size_t const &num_robots,
					CoverageSystem &env) :
				params_{params},
				num_robots_{num_robots},
				env_{env} {

					robot_global_positions_ = env_.GetRobotPositions();
					actions_.resize(num_robots_);
					goals_ = robot_global_positions_;
					voronoi_mass_.resize(num_robots_, 0);
					voronoi_ = Voronoi(robot_global_positions_, env_.GetWorldIDF(), Point2(params_.pWorldMapSize, params_.pWorldMapSize), params_.pResolution);
					ComputeGoals();
				}

			PointVector GetActions() { return actions_; }

			auto GetGoals() { return goals_; }

			auto &GetVoronoi() { return voronoi_; }

			void ComputeGoals() {
				voronoi_.UpdateSites(robot_global_positions_);
				auto voronoi_cells = voronoi_.GetVoronoiCells();
				for(size_t iRobot = 0; iRobot < num_robots_; ++iRobot) {
					goals_[iRobot] = voronoi_cells[iRobot].centroid();
					voronoi_mass_[iRobot] = voronoi_cells[iRobot].mass();
				}
			}

			bool Step() {
				continue_flag_ = false;
				robot_global_positions_ = env_.GetRobotPositions();
				ComputeGoals();
				auto voronoi_cells = voronoi_.GetVoronoiCells();
				for(size_t iRobot = 0; iRobot < num_robots_; ++iRobot) {
					actions_[iRobot] = Point2(0, 0);
					Point2 diff = goals_[iRobot] - robot_global_positions_[iRobot];
					double dist = diff.norm();
					if(dist < 0.1 * params_.pResolution) {
						continue;
					}
					if(env_.CheckOscillation(iRobot)) {
						continue;
					}
					double speed = dist / params_.pTimeStep;
					/* double speed = 2 * dist * voronoi_mass_[iRobot]; */
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
#endif /* COVERAGECONTROL_ALGORITHMS_LLOYD_GLOBAL_ONLINE_H_ */
