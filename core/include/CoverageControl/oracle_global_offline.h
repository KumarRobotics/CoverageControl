/**
 *
 **/

#ifndef COVERAGECONTROL_ORACLE_GLOBAL_OFFLINE_H_
#define COVERAGECONTROL_ORACLE_GLOBAL_OFFLINE_H_

#include <vector>
#include <fstream>
#include <iostream>
#include <random>
#include <omp.h>
#define EIGEN_NO_CUDA // Don't use eigen's cuda facility
#include <Eigen/Dense> // Eigen is used for maps

#include "parameters.h"
#include "typedefs.h"
#include "coverage_system.h"
#include <lsap/Hungarian.h>

namespace CoverageControl {

	class OracleGlobalOffline {
		private:
			Parameters const params_;
			size_t num_robots_ = 0;
			CoverageSystem &env_;
			std::vector <VoronoiCell> voronoi_cells_;
			PointVector robot_global_positions_;
			PointVector goals_, actions_;
			std::vector <std::vector<double>> cost_matrix_;

		public:
			OracleGlobalOffline(
					Parameters const &params,
					size_t const &num_robots,
					CoverageSystem &env) :
				params_{params},
				num_robots_{num_robots},
				env_{env}{

				cost_matrix_.resize(num_robots_, std::vector<double>(num_robots_));
				voronoi_cells_.resize(num_robots_);

				robot_global_positions_ = env_.GetRobotPositions();
				actions_ = PointVector(num_robots_);
				Initialize();
			}

			PointVector GetActions() { return actions_; }

			void Initialize() {
				voronoi_cells_ = env_.LloydOracle(params_.pLloydNumTries, params_.pLloydMaxIterations, num_robots_, env_.GetWorldIDF(), params_.pWorldMapSize, params_.pResolution);
#pragma omp parallel for num_threads(num_robots_)
				for(size_t iRobot = 0; iRobot < num_robots_; ++iRobot) {
					for(size_t jCentroid = 0; jCentroid < voronoi_cells_.size(); ++jCentroid) {
						cost_matrix_[iRobot][jCentroid] = (robot_global_positions_[iRobot] - voronoi_cells_[jCentroid].centroid).norm();
					}
				}
				HungarianAlgorithm HungAlgo;
				std::vector<int> assignment;
				HungAlgo.Solve(cost_matrix_, assignment);

				goals_.reserve(num_robots_);
				for(size_t iRobot = 0; iRobot < num_robots_; ++iRobot) {
					goals_[iRobot] = voronoi_cells_[assignment[iRobot]].centroid;
				}
			}

			bool Step() {
				bool cont_flag = true;
				robot_global_positions_ = env_.GetRobotPositions();
				for(size_t iRobot = 0; iRobot < num_robots_; ++iRobot) {
					actions_[iRobot] = Point2(0, 0);
					cont_flag = false;
					Point2 diff = goals_[iRobot] - robot_global_positions_[iRobot];
					if(diff.squaredNorm() > params_.pResolution * params_.pResolution) {
						double dist = diff.norm();
						double speed = dist / params_.pTimeStep;
						if(speed <= kEps) {
							continue;
						}
						speed = std::min(params_.pMaxRobotSpeed, speed);
						Point2 direction(diff);
						direction.normalize();
						actions_[iRobot] = speed * direction;
						if(env_.StepControl(iRobot, direction, speed)) {
							std::cerr << "Control incorrect\n";
							return 1;
						}
						cont_flag = true;
					}
				}
				return cont_flag;
			}
	};

} /* namespace CoverageControl */
#endif /* COVERAGECONTROL_ORACLE_GLOBAL_OFFLINE_H_ */
