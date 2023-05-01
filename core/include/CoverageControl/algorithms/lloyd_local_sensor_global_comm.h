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
#include "../map_utils.h"
#include "lloyd_algorithms.h"
#include <lsap/Hungarian.h>

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
					voronoi_ = Voronoi(robot_global_positions_, env_.GetWorldIDF(), Point2(params_.pWorldMapSize, params_.pWorldMapSize), params_.pResolution);
					ComputeGoals();
				}

			PointVector GetActions() { return actions_; }

			auto GetGoals() { return goals_; }

			auto &GetVoronoi() { return voronoi_; }

			void ComputeGoals() {
				voronoi_.UpdateSites(robot_global_positions_);
#pragma omp parallel for num_threads(num_robots_)
				for(size_t iRobot = 0; iRobot < num_robots_; ++iRobot) {
					auto const &pos = robot_global_positions_[iRobot];
					MapUtils::MapBounds index, offset;
					MapUtils::ComputeOffsets(params_.pResolution, pos, params_.pLocalMapSize, params_.pWorldMapSize, index, offset);
					auto robot_map = env_.GetRobotMap(iRobot);
					auto robot_local_map = robot_map.block(index.left + offset.left, index.bottom + offset.bottom, offset.width, offset.height);
					Point2 map_translation((index.left + offset.left) * params_.pResolution, (index.bottom + offset.bottom) * params_.pResolution);

					PointVector robot_neighbors_pos;
					for(size_t jRobot = 0; jRobot < num_robots_; ++jRobot) {
						if(iRobot == jRobot) {
							continue;
						}
						robot_neighbors_pos.push_back(robot_global_positions_[jRobot]);
					}
					PointVector robot_positions(robot_neighbors_pos.size() + 1);

					robot_positions[0] = robot_global_positions_[iRobot] - map_translation;
					/* std::cout << "Voronoi: " << robot_positions[0][0] << " " << robot_positions[0][1] << std::endl; */
					int count = 1;
					for(auto const &pos:robot_neighbors_pos) {
						robot_positions[count] = pos - map_translation;
						++count;
					}
					Point2 map_size(offset.width, offset.height);
					Voronoi voronoi(robot_positions, robot_local_map, map_size, params_.pResolution, true, 0);
					auto vcell = voronoi.GetVoronoiCell();
					voronoi_mass_[iRobot] = vcell.mass();
					goals_[iRobot] = vcell.centroid() + robot_positions[0] + map_translation;
					if(goals_[iRobot][0] < 0 or goals_[iRobot][0] > params_.pWorldMapSize or goals_[iRobot][1] < 0 or goals_[iRobot][1] > params_.pWorldMapSize) {
						std::cout << "Goal out of bounds: " << goals_[iRobot][0] << " " << goals_[iRobot][1] << std::endl;
						std::cout << robot_global_positions_[iRobot][0] << " " << robot_global_positions_[iRobot][1] << std::endl;
						std::cout << vcell.centroid()[0] << " " << vcell.centroid()[1] << std::endl;
						std::cout << map_translation[0] << " " << map_translation[1] << std::endl;
						std::cout << robot_local_map.sum() << std::endl;
						throw std::runtime_error("Goal out of bounds: this should not have happened for convex environments");
					}
				}
			}

			bool Step() {
				continue_flag_ = false;
				robot_global_positions_ = env_.GetRobotPositions();
				ComputeGoals();
				for(size_t iRobot = 0; iRobot < num_robots_; ++iRobot) {
					actions_[iRobot] = Point2(0, 0);
					Point2 diff = goals_[iRobot] - robot_global_positions_[iRobot];
					double dist = diff.norm();
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
