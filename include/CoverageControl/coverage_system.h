/**
 * A class for creating world and robots
 *
 **/

#ifndef COVERAGECONTROL_COVERAGE_SYSTEM_H_
#define COVERAGECONTROL_COVERAGE_SYSTEM_H_

#include <vector>
#include <fstream>
#include <iostream>
#include <random>
#define EIGEN_NO_CUDA // Don't use eigen's cuda facility
#include <Eigen/Dense> // Eigen is used for maps

#include "constants.h"
#include "parameters.h"
#include "typedefs.h"
#include "bivariate_normal_distribution.h"
#include "map_utils.h"

namespace CoverageControl {

	class CoverageSystem {
		private:
			WorldIDF world_idf_;
			size_t num_robots_ = 0;
			std::vector <RobotModel> robots_;

		public:
			// Initialize IDF with num_gaussians distributions
			// Initialize num_robots with random start positions
			CoverageSystem(int const num_gaussians, int const num_robots) {
				std::srand(0);
				std::random_device rd;  //Will be used to obtain a seed for the random number engine
				std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
				std::uniform_real_distribution<> distrib(0, pWorldMapSize * pResolution);
				std::uniform_real_distribution<> distrib_var(pMinVariance, pMaxVariance);
				std::uniform_real_distribution<> distrib_peak(pMinPeak, pMaxPeak);
				for(int i = 0; i < num_gaussians; ++i) {
					Point2 mean(distrib(gen), distrib(gen));
					double var(distrib_var(gen));
					double peak(distrib_peak(gen));
					BivariateNormalDistribution dist(mean, var, peak);
					world_idf_.AddNormalDistribution(dist);
				}
				world_idf_.GenerateMapCuda();

				robots_.reserve(num_robots);
				for(int i = 0; i < num_robots; ++i) {
					Point2 start_pos(distrib(gen), distrib(gen));
					robots_.push_back(RobotModel(start_pos, world_idf_));
				}
				num_robots_ = robots_.size();
			}

			// Can create the system using a previously created WorldIDF object
			CoverageSystem(WorldIDF const &world_idf) {
				SetWorldIDF(world_idf);
			}

			CoverageSystem(WorldIDF const &world_idf, std::vector <Point2> const &robot_positions) {
				SetWorldIDF(world_idf);
				robots_.reserve(robot_positions.size());
				for(auto const &pos:robot_positions) {
					robots_.push_back(RobotModel(pos, world_idf_));
				}
				num_robots_ = robots_.size();
			}

			CoverageSystem(std::vector <BivariateNormalDistribution> const &dists, std::vector <Point2> const &robot_positions) {
				world_idf_.AddNormalDistribution(dists);
				num_robots_ = robot_positions.size();
				robots_.reserve(num_robots_);
				for(auto const &pos:robot_positions) {
					robots_.push_back(RobotModel(pos, world_idf_));
				}

			}

			void SetWorldIDF(WorldIDF const &world_idf) { world_idf_ = world_idf; }

			void StepControl(std::vector<Point2> const &directions, std::vector<double> speeds) {
				if((directions.size() != num_robots_) and (speeds.size() != num_robots_)) {
					throw std::length_error{"The size of the vectors don't match with the number of robots"};
				}
				for(size_t i = 0; i < num_robots_; ++i) {
					if(robots_[i].StepControl(directions[i], speeds[i])) {
						std::cerr << "Control incorrect\n";
					}
				}
			}

			void UpdateRobotPositions(std::vector<Point2> const &positions) {
				if(positions.size() != num_robots_) {
					throw std::length_error{"The size of the positions don't match with the number of robots"};
				}
				for(size_t i = 0; i < num_robots_; ++i) {
					robots_[i].UpdateRobotPosition(positions[i]);
				}
			}

			PointVector GetRobotPositions() const {
				PointVector robot_positions;
				robot_positions.reserve(num_robots_);
				for(auto const& robot:robots_) {
					robot_positions.push_back(robot.GetGlobalCurrentPosition());
				}
				return robot_positions;
			}

			const MapType& GetWorldIDF() const { return world_idf_.GetWorldMap(); }

			const MapType& GetRobotLocalMap(size_t const idx) {
				if(idx < num_robots_) {
					return robots_[idx].GetRobotLocalMap();
				} else {
					throw std::out_of_range{"Robot index more than the number of robots"};
				}
			}

			const MapType& GetRobotSensorView(size_t const idx) {
				if(idx < num_robots_) {
					return robots_[idx].GetSensorView();
				} else {
					throw std::out_of_range{"Robot index more than the number of robots"};
				}
			}
	};

} /* namespace CoverageControl */
#endif /* _COVERAGECONTROL_COVERAGE_SYSTEM_H_ */
