/**
 * A class for creating world and robots
 *
 **/

#ifndef COVERAGECONTROL_COVERAGE_SYSTEM_H_
#define COVERAGECONTROL_COVERAGE_SYSTEM_H_

#include <vector>
#include <fstream>
#include <iostream>
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
				for(size_t i = 0; i < num_gaussians; ++i) {
					Point2 mean(distrib(gen), distrib(gen));
					double var(distrib_var(gen));
					double peak(distrib_peak(gen));
					BivariateNormalDistribution dist(mean, var, peak);
					world_idf_.AddNormalDistribution(dist);
				}
				world_idf_.GenerateMapCuda();

				robots_.reserve(num_robots);
				for(size_t i = 0; i < num_robots; ++i) {
					Point2 start_pos(distrib(gen), distrib(gen));
					robots_.push_back(RobotModel(start_pos, world_idf_));
				}
			}

			// Can create the system using a previously created WorldIDF object and RobotModel objects
			CoverageSystem(WorldIDF const &world_idf, std::vector <RobotModel> const &robots) {
				SetWorldIDF(world_idf);
				SetRobots(robots);
			}

			void SetWorldIDF(WorldIDF const &world_idf) { world_idf_ = world_idf; }
			void SetRobots(std::vector <RobotModel> const &robots) { robots_ = robots; }


	};

} /* namespace CoverageControl */
#endif /* _COVERAGECONTROL_COVERAGE_SYSTEM_H_ */
