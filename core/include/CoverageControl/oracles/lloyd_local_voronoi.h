/**
 *
 **/

#ifndef COVERAGECONTROL_LLOYD_LOCAL_VORONOI_H_
#define COVERAGECONTROL_LLOYD_LOCAL_VORONOI_H_

#include <vector>
#include <fstream>
#include <iostream>
#include <random>
#include <omp.h>
#define EIGEN_NO_CUDA // Don't use eigen's cuda facility
#include <Eigen/Dense> // Eigen is used for maps

#include "../parameters.h"
#include "../typedefs.h"
#include "../coverage_system.h"
#include "../lloyd_algorithms.h"
#include <lsap/Hungarian.h>

namespace CoverageControl {

	class LloydLocalVoronoi {
		private:
			Parameters const params_;
			size_t num_robots_ = 0;
			CoverageSystem &env_;
			Voronoi voronoi_;
			std::vector <VoronoiCell> voronoi_cells_;
			PointVector robot_global_positions_;
			PointVector goals_, actions_;
			std::vector <std::vector<double>> cost_matrix_;
			MapType oracle_map_;

		public:
			LloydLocalVoronoi(
					Parameters const &params,
					size_t const &num_robots,
					CoverageSystem &env) :
				params_{params},
				num_robots_{num_robots},
				env_{env}{

				cost_matrix_.resize(num_robots_, std::vector<double>(num_robots_));
				voronoi_cells_.resize(num_robots_);

				robot_global_positions_ = env_.GetRobotPositions();
				actions_.resize(num_robots_);
				goals_.resize(num_robots_);

				// The oracle map is designed to store the pixels seen by any robot
				oracle_map_ = MapType::Constant(params_.pWorldMapSize, params_.pWorldMapSize, params_.pUnknownImportance * params_.pNorm);
				UpdateOracleMap();
			}

			PointVector GetActions() { return actions_; }

			MapType const& GetOracleMap() { return oracle_map_; }

			void SetGoals(PointVector const &goals) {
				goals_ = goals;
			}

			auto GetGoals() { return goals_; }

			std::vector<VoronoiCell> GetVoronoiCells() { return voronoi_cells_; }

			void UpdateOracleMap() {
				robot_global_positions_ = env_.GetRobotPositions();
				for(size_t i = 0; i < num_robots_; ++i) {
					MapUtils::MapBounds index, offset;
					MapUtils::ComputeOffsets(params_.pResolution, robot_global_positions_[i], params_.pSensorSize, params_.pWorldMapSize, index, offset);
					oracle_map_.block(index.left + offset.left, index.bottom + offset.bottom, offset.width, offset.height) = env_.GetRobotSensorView(i).block(offset.left, offset.bottom, offset.width, offset.height);
				}
			}

			void ComputeGoals() {
#pragma omp parallel for num_threads(num_robots_)
				for(size_t iRobot = 0; iRobot < num_robots_; ++iRobot) {
					for(size_t jCentroid = 0; jCentroid < voronoi_cells_.size(); ++jCentroid) {
						cost_matrix_[iRobot][jCentroid] = (robot_global_positions_[iRobot] - voronoi_cells_[jCentroid].centroid).norm();
					}
				}
				HungarianAlgorithm HungAlgo;
				std::vector<int> assignment;
				HungAlgo.Solve(cost_matrix_, assignment);

				auto ordered_voronoi_cells = voronoi_cells_;
				for(size_t iRobot = 0; iRobot < num_robots_; ++iRobot) {
					goals_[iRobot] = voronoi_cells_[assignment[iRobot]].centroid;
					ordered_voronoi_cells[iRobot] = voronoi_cells_[assignment[iRobot]];
				}
				voronoi_cells_ = ordered_voronoi_cells;
			}

			bool Step(int const num_steps = 0) {
#pragma omp parallel for num_threads(num_robots_)
				for(size_t iRobot = 0; iRobot < num_robots_; ++iRobot) {
					auto robot_local_map = env_.GetRobotLocalMap(iRobot);
					auto robot_neighbors_pos = env_.GetRobotsInCommunication(iRobot);
					PointVector robot_positions(robot_neighbors_pos.size() + 1);
					/* Point2 map_translation(params_.pLocalMapSize * params_.pResolution/2., params_.pLocalMapSize * params_.pResolution/2.); */
					Point2 map_translation((index.left + offset.left) * params_.pResolution, (index.bottom + offset.bottom) * params_.pResolution);
					robot_positions[0] = robot_global_positions_[iRobot] - map_translation;
					int count = 1;
					for(auto const &pos:robot_neighbors_pos) {
						robot_positions[count] = pos - map_translation;
						++count;
					}
					Voronoi voronoi(robot_positions, robot_local_map, params_.pLocalMapSize, params_.pResolution, true, 0);
					voronoi_cells_[iRobot] = voronoi.GetVoronoiCell();
					/* goals_[iRobot] = voronoi_cells_[iRobot].centroid + robot_global_positions_[iRobot] + map_translation; */
					goals_[iRobot] = vcell.centroid + map_translation;
				}
				bool cont_flag = true;
				for(int i = 0; i < num_steps; ++i) {
					cont_flag = env_.StepRobotsToGoals(goals_, actions_);
					UpdateOracleMap();
					if(cont_flag == false) {
						break;
					}
				}
				return cont_flag;
			}

	};

} /* namespace CoverageControl */
#endif /* COVERAGECONTROL_LLOYD_LOCAL_VORONOI_H_ */
