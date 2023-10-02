/**
 * The coverage control algorithm uses Lloyd's algorithm on accumulated local map of individual robots, i.e., a robot has knowledge only about the regions it has visited.
 * Communication radius is not considered
 * The algorithm is online---it takes localized actions based on the current robot positions.
 **/

#ifndef COVERAGECONTROL_ALGORITHMS_LLOYD_LOCAL_SENSOR_GLOBAL_COMM_H_
#define COVERAGECONTROL_ALGORITHMS_LLOYD_LOCAL_SENSOR_GLOBAL_COMM_H_

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

namespace CoverageControl {

	class LloydLocalSensorGlobalComm {
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
			LloydLocalSensorGlobalComm(
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
					voronoi_ = Voronoi(robot_global_positions_, env_.GetSystemExploredIDFMap(), Point2(params_.pWorldMapSize, params_.pWorldMapSize), params_.pResolution);
					ComputeGoals();
				}

			PointVector GetActions() { return actions_; }

			auto GetGoals() { return goals_; }

			auto &GetVoronoi() { return voronoi_; }

			void ComputeGoals() {
				voronoi_ = Voronoi(robot_global_positions_, env_.GetSystemExploredIDFMap(), Point2(params_.pWorldMapSize, params_.pWorldMapSize), params_.pResolution);
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
					/* if(dist < 0.1 * params_.pResolution) { */
					/* 	continue; */
					/* } */
					if(dist < kEps) {
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
#endif /* COVERAGECONTROL_ALGORITHMS_LLOYD_LOCAL_SENSOR_GLOBAL_COMM_H_ */
