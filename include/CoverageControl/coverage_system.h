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
#include <omp.h>
#define EIGEN_NO_CUDA // Don't use eigen's cuda facility
#include <Eigen/Dense> // Eigen is used for maps

#include "constants.h"
#include "parameters.h"
#include "typedefs.h"
#include "bivariate_normal_distribution.h"
#include "map_utils.h"
#include "voronoi.h"

namespace CoverageControl {

	class CoverageSystem {
		private:
			Parameters const params_;
			WorldIDF world_idf_;
			size_t num_robots_ = 0;
			std::vector <RobotModel> robots_;
			MapType communication_map_;
			Voronoi voronoi_;
			double normalization_factor_ = 0;
			std::vector <VoronoiCell> voronoi_cells_;
			std::random_device rd_;  //Will be used to obtain a seed for the random number engine
			std::mt19937 gen_;
			std::uniform_real_distribution<> distrib_pts_;

		public:
			// Initialize IDF with num_gaussians distributions
			// Initialize num_robots with random start positions
			CoverageSystem(Parameters const &params, int const num_gaussians, int const num_robots) : params_{params}, world_idf_{WorldIDF(params_)}{
				std::srand(0);
				gen_ = std::mt19937(rd_()); //Standard mersenne_twister_engine seeded with rd_()
				distrib_pts_ = std::uniform_real_distribution<>(0, params_.pWorldMapSize * params_.pResolution);
				std::uniform_real_distribution<> distrib_var(params_.pMinVariance, params_.pMaxVariance);
				std::uniform_real_distribution<> distrib_peak(params_.pMinPeak, params_.pMaxPeak);
				for(int i = 0; i < num_gaussians; ++i) {
					Point2 mean(distrib_pts_(gen_), distrib_pts_(gen_));
					double var(distrib_var(gen_));
					double peak(distrib_peak(gen_));
					BivariateNormalDistribution dist(mean, var, peak);
					world_idf_.AddNormalDistribution(dist);
				}
				world_idf_.GenerateMapCuda();
				normalization_factor_ = world_idf_.GetNormalizationFactor();

				robots_.reserve(num_robots);
				for(int i = 0; i < num_robots; ++i) {
					Point2 start_pos(distrib_pts_(gen_), distrib_pts_(gen_));
					robots_.push_back(RobotModel(params_, start_pos, world_idf_));
				}
				num_robots_ = robots_.size();
			}

			CoverageSystem(Parameters const &params, WorldIDF const &world_idf, std::vector <Point2> const &robot_positions) : params_{params}, world_idf_{WorldIDF(params_)}{
				SetWorldIDF(world_idf);
				normalization_factor_ = world_idf_.GetNormalizationFactor();
				robots_.reserve(robot_positions.size());
				for(auto const &pos:robot_positions) {
					robots_.push_back(RobotModel(params_, pos, world_idf_));
				}
				num_robots_ = robots_.size();
			}

			CoverageSystem(Parameters const &params, std::vector <BivariateNormalDistribution> const &dists, std::vector <Point2> const &robot_positions) : params_{params}, world_idf_{WorldIDF(params_)}{
				world_idf_.AddNormalDistribution(dists);
				world_idf_.GenerateMapCuda();
				normalization_factor_ = world_idf_.GetNormalizationFactor();
				num_robots_ = robot_positions.size();
				robots_.reserve(num_robots_);
				for(auto const &pos:robot_positions) {
					robots_.push_back(RobotModel(params_, pos, world_idf_));
				}

			}

			void SetWorldIDF(WorldIDF const &world_idf) { world_idf_ = world_idf;
				normalization_factor_ = world_idf_.GetNormalizationFactor();
			}

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

			const MapType& GetRobotLocalMap(size_t const id) {
				if(id < num_robots_) {
					return robots_[id].GetRobotLocalMap();
				} else {
					throw std::out_of_range{"Robot index more than the number of robots"};
				}
			}

			const MapType& GetRobotSensorView(size_t const id) {
				if(id < num_robots_) {
					return robots_[id].GetSensorView();
				} else {
					throw std::out_of_range{"Robot index more than the number of robots"};
				}
			}

			const MapType& GetCommunicationMap(size_t const id) {
				communication_map_ = MapType::Zero(params_.pLocalMapSize, params_.pLocalMapSize);
				if(id < num_robots_) {
					double comm_range_sqr = params_.pCommunicationRange * params_.pCommunicationRange;
					auto robot_positions = GetRobotPositions();
					for(size_t i = 0; i < num_robots_; ++i) {
						if(id == i) {
							continue;
						}
						auto relative_pos = robot_positions[i] - robot_positions[id];
						if(relative_pos.NormSqr() <= comm_range_sqr) {
							int pos_idx, pos_idy;
							MapUtils::GetClosestGridCoordinate(params_.pResolution, relative_pos, pos_idx, pos_idy);
							communication_map_(pos_idx, pos_idy) = 1;
						}
					}
				} else {
					throw std::out_of_range{"Robot index more than the number of robots"};
				}
				return communication_map_;
			}

			void ComputeVoronoiCells() {
				voronoi_ = Voronoi(GetRobotPositions(), world_idf_.GetWorldMap(), params_.pWorldMapSize, params_.pResolution);
				voronoi_cells_ = voronoi_.GetVoronoiCells();
				for(size_t i = 0; i < num_robots_; ++i) {
					robots_[i].SetVoronoiCell(voronoi_cells_[i]);
				}
			}

			bool StepLloyd() {
				bool cont_flag = false;
				ComputeVoronoiCells();
				for(size_t i = 0; i < num_robots_; ++i) {
					auto diff = voronoi_cells_[i].centroid - voronoi_cells_[i].site;
					double speed = 2 * voronoi_cells_[i].mass * diff.Norm();
					/* double speed = diff.Norm() / params_.pTimeStep; */
					if(speed < kEps) {
						continue;
					}
					cont_flag = true;
					speed = std::min(params_.pMaxRobotSpeed, speed);
					auto direction = diff; direction.Normalize();
					if(robots_[i].StepControl(direction, speed)) {
						std::cerr << "Control incorrect\n";
					}
				}
				return cont_flag;
			}

			void Lloyd() {
				bool cont_flag = true;
				for(int iStep = 0; iStep < params_.pEpisodeSteps and cont_flag == true; ++iStep) {
					std::cout << "iStep: " << iStep << std::endl;
					cont_flag = StepLloyd();
				}
			}

			/* void LloydOffline() { */
			/* 	std::vector <std::vector <Point2>> all_sites; */
			/* 	std::vector <double> all_mass; */
/* #pragma omp parallel for */
			/* 	for(int i = 0; i < params_.pNumLloydOfflineTries; ++i) { */
			/* 		for(int iRobot = 0; iRobot < num_robots_; ++iRobot) { */
			/* 			all_sites[i][iRobot] = (Point2(distrib_pts_(gen_), distrib_pts_(gen_))); */
			/* 		} */
			/* 		bool cont_flag = true; */
			/* 		for(int iSteps = 0; iSteps < params_.pLloydOfflineMaxIteration and cont_flag == true; ++iSteps) { */
			/* 			Voronoi voronoi(all_sites[i], world_idf_.GetWorldMap(), params_.pWorldMapSize, params_.pResolution); */
			/* 			auto voronoi_cells = voronoi.GetVoronoiCells(); */
			/* 			cont_flag = false; */
			/* 			for(size_t iRobot = 0; iRobot < num_robots_; ++iRobot) { */
			/* 				auto diff = voronoi_cells[iRobot].centroid - voronoi_cells[iRobot].site; */
			/* 				if(diff.Norm() < kEps) { */
			/* 					continue; */
			/* 				} */
			/* 				cont_flag = true; */
			/* 				all_sites[i][iRobot] = voronoi_cells[iRobot].centroid; */
			/* 			} */
			/* 		} */
			/* 	} */
			/* } */

			auto GetVoronoiCells() {
				return voronoi_cells_;
			}

			/* auto GetVoronoiEdges () { */
			/* 	return voronoi_.GetVoronoiEdges(); */
			/* } */

			double GetNormalizationFactor() {
				normalization_factor_=world_idf_.GetNormalizationFactor();
				return normalization_factor_;
			}
	};

} /* namespace CoverageControl */
#endif /* _COVERAGECONTROL_COVERAGE_SYSTEM_H_ */
