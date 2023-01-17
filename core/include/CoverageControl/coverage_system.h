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
#include "lloyd_algorithms.h"
#include <lsap/Hungarian.h>

namespace CoverageControl {

	class CoverageSystem {
		private:
			Parameters const params_;
			WorldIDF world_idf_;
			MapType oracle_map_;
			size_t num_robots_ = 0;
			std::vector <RobotModel> robots_;
			MapType communication_map_;
			double normalization_factor_ = 0;
			Voronoi voronoi_;
			std::vector <VoronoiCell> voronoi_cells_;
			std::random_device rd_;  //Will be used to obtain a seed for the random number engine
			std::mt19937 gen_;
			std::uniform_real_distribution<> distrib_pts_;
			std::vector <std::vector<double>> cost_matrix_;
			PointVector robot_global_positions_;

		public:
			// Initialize IDF with num_gaussians distributions
			// Initialize num_robots with random start positions
			CoverageSystem(
					Parameters const &params,
					int const num_gaussians,
					int const num_robots) :
				params_{params},
				world_idf_{WorldIDF(params_)}{
				// Generate Bivariate Normal Distribution from random numbers
				std::srand(0);
				gen_ = std::mt19937(rd_()); //Standard mersenne_twister_engine seeded with rd_()
				distrib_pts_ = std::uniform_real_distribution<>(0, params_.pWorldMapSize * params_.pResolution);
				std::uniform_real_distribution<> distrib_var(params_.pMinSigma, params_.pMaxSigma);
				std::uniform_real_distribution<> distrib_peak(params_.pMinPeak, params_.pMaxPeak);
				for(int i = 0; i < num_gaussians; ++i) {
					Point2 mean(distrib_pts_(gen_), distrib_pts_(gen_));
					double var(distrib_var(gen_));
					double peak(distrib_peak(gen_));
					BivariateNormalDistribution dist(mean, var, peak);
					world_idf_.AddNormalDistribution(dist);
				}

				// Generate the world map using Cuda
				world_idf_.GenerateMapCuda();
				normalization_factor_ = world_idf_.GetNormalizationFactor();

				std::uniform_real_distribution<> robot_pos_dist (0, params_.pRobotInitDist);
				robots_.reserve(num_robots);
				for(int i = 0; i < num_robots; ++i) {
					Point2 start_pos(robot_pos_dist(gen_), robot_pos_dist(gen_));
					robots_.push_back(RobotModel(params_, start_pos, world_idf_));
				}
				num_robots_ = robots_.size();

				cost_matrix_.resize(num_robots_, std::vector<double>(num_robots_));
				voronoi_cells_.resize(num_robots_);

				// The oracle map is designed to store the pixels seen by any robot
				oracle_map_ = MapType::Constant(params_.pWorldMapSize, params_.pWorldMapSize, params_.pUnknownImportance * params_.pNorm);
				robot_global_positions_.resize(num_robots_);
				UpdateOracleMap();
				for(size_t iRobot = 0; iRobot < num_robots_; ++iRobot) {
					robot_global_positions_[iRobot] = robots_[iRobot].GetGlobalCurrentPosition();
				}
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
				num_robots_ = robots_.size();

			}

			void SetWorldIDF(WorldIDF const &world_idf) { world_idf_ = world_idf;
				normalization_factor_ = world_idf_.GetNormalizationFactor();
			}

			bool StepControl(size_t robot_id, Point2 const &direction, double const speed) {
				if(robots_[robot_id].StepControl(direction, speed)) {
						std::cerr << "Control incorrect\n";
						return 1;
				}
				robot_global_positions_[robot_id] = robots_[robot_id].GetGlobalCurrentPosition();
				return 0;
			}

			/* void StepControl(std::vector<Point2> const &directions, std::vector<double> speeds) { */
			/* 	if((directions.size() != num_robots_) and (speeds.size() != num_robots_)) { */
			/* 		throw std::length_error{"The size of the vectors don't match with the number of robots"}; */
			/* 	} */
			/* 	for(size_t i = 0; i < num_robots_; ++i) { */
			/* 		if(robots_[i].StepControl(directions[i], speeds[i])) { */
			/* 			std::cerr << "Control incorrect\n"; */
			/* 		} */
			/* 		robot_global_positions_[i] = robots_[i].GetGlobalCurrentPosition(); */

			/* 	} */
			/* } */

			void UpdateRobotPositions(std::vector<Point2> const &positions) {
				if(positions.size() != num_robots_) {
					throw std::length_error{"The size of the positions don't match with the number of robots"};
				}
				for(size_t i = 0; i < num_robots_; ++i) {
					robots_[i].UpdateRobotPosition(positions[i]);
				}
			}

			PointVector GetRobotPositions() {
				for(size_t iRobot = 0; iRobot < num_robots_; ++iRobot) {
					robot_global_positions_[iRobot] = robots_[iRobot].GetGlobalCurrentPosition();
				}
				return robot_global_positions_;
			}
			Point2 GetRobotPosition(int const robot_id) { return robots_[robot_id].GetGlobalCurrentPosition(); }

			MapType const& GetWorldIDF() const { return world_idf_.GetWorldMap(); }

			MapType const& GetRobotLocalMap(size_t const id) {
				if(id < num_robots_) {
					return robots_[id].GetRobotLocalMap();
				} else {
					throw std::out_of_range{"Robot index more than the number of robots"};
				}
			}

			MapTypeBool const& GetRobotExplorationMap(size_t const id) {
				if(id < num_robots_) {
					return robots_[id].GetExplorationMap();
				} else {
					throw std::out_of_range{"Robot index more than the number of robots"};
				}
			}

			MapType const& GetRobotObstacleMap(size_t const id) {
				if(id < num_robots_) {
					return robots_[id].GetObstacleMap();
				} else {
					throw std::out_of_range{"Robot index more than the number of robots"};
				}
			}

			MapType const& GetOracleMap() {
				return oracle_map_;
			}

			MapType const& GetRobotSensorView(size_t const id) {
				if(id < num_robots_) {
					return robots_[id].GetSensorView();
				} else {
					throw std::out_of_range{"Robot index more than the number of robots"};
				}
			}

			// Get robots within a square communication range
			auto GetRobotsInCommunication(size_t const id) {
				PointVector robot_neighbors_pos;
				if(id < num_robots_) {
					/* double comm_range_sqr = params_.pCommunicationRange * params_.pCommunicationRange; */
					for(size_t i = 0; i < num_robots_; ++i) {
						if(id == i) {
							continue;
						}
						Point2 relative_pos = robot_global_positions_[i] - robot_global_positions_[id];
						if(relative_pos.x() < params_.pCommunicationRange and
								relative_pos.x() > -params_.pCommunicationRange and
								relative_pos.y() < params_.pCommunicationRange and
								relative_pos.y() > -params_.pCommunicationRange) {
							robot_neighbors_pos.push_back(relative_pos);
						}
						/* if(relative_pos.squaredNorm() <= comm_range_sqr) { */
						/* 	robot_neighbors_pos.push_back(relative_pos); */
						/* } */
					}
				} else {
					throw std::out_of_range{"Robot index more than the number of robots"};
				}
				return robot_neighbors_pos;
			}

			MapType const& GetCommunicationMap(size_t const id) {
				communication_map_ = MapType::Zero(params_.pLocalMapSize, params_.pLocalMapSize);
				auto robot_neighbors_pos = GetRobotsInCommunication(id);
				/* std::cout << "Num neigh: " << robot_neighbors_pos.size() << std::endl; */
				double comm_scale = (params_.pCommunicationRange * 2.) / params_.pLocalMapSize;
				Point2 map_translation(params_.pLocalMapSize * comm_scale * params_.pResolution/2., params_.pLocalMapSize * comm_scale * params_.pResolution/2.);
				for(Point2 const& relative_pos:robot_neighbors_pos) {
					Point2 map_pos = relative_pos + map_translation;
					int pos_idx, pos_idy;
					MapUtils::GetClosestGridCoordinate(params_.pResolution * comm_scale, map_pos, pos_idx, pos_idy);
					if(pos_idx < params_.pLocalMapSize and pos_idy < params_.pLocalMapSize and pos_idx >= 0 and pos_idy >= 0) {
						communication_map_(pos_idx, pos_idy) = 1;
					}
				}
				return communication_map_;
			}

			void ComputeVoronoiCells() {
				GetRobotPositions();
				voronoi_ = Voronoi(robot_global_positions_, GetWorldIDF(), params_.pWorldMapSize, params_.pResolution);
				voronoi_cells_ = voronoi_.GetVoronoiCells();
			}

			bool StepRobotToPoint(int const robot_id, Point2 const &goal, double const speed_factor = 1) {
				Point2 curr_pos = robots_[robot_id].GetGlobalCurrentPosition();
				Point2 diff = goal - curr_pos;
				double dist = diff.norm();
				double speed = speed_factor * dist / params_.pTimeStep;
				if(speed <= kEps) {
					return 0;
				}
				speed = std::min(params_.pMaxRobotSpeed, speed);
				Point2 direction(diff);
				direction.normalize();
				/* if(robot_id == 0) { */
				/* 	std::cout << "Pos: " << curr_pos.x() << " " << curr_pos.y() << std::endl; */
				/* 	std::cout << "Goal: " << goal.x() << " " << goal.y() << std::endl; */
				/* 	std::cout << "Direction: " << direction.x() << " " << direction.y() << std::endl; */
				/* 	std::cout << "Speed: " << speed << std::endl; */
				/* } */
				if(robots_[robot_id].StepControl(direction, speed)) {
					std::cerr << "Control incorrect\n";
					return 1;
				}
				robot_global_positions_[robot_id] = robots_[robot_id].GetGlobalCurrentPosition();
				return 0;
			}

			bool StepLloyd() {
				bool cont_flag = false;
				ComputeVoronoiCells();
				for(size_t i = 0; i < num_robots_; ++i) {
					Point2 diff = voronoi_cells_[i].centroid - voronoi_cells_[i].site;
					double dist = diff.norm();
					double speed = 2 * voronoi_cells_[i].mass * dist;
					/* double speed = diff.norm() / params_.pTimeStep; */
					if(dist <= params_.pResolution) {
						speed = dist / params_.pTimeStep;
					}
					if(speed <= kEps) {
						continue;
					}
					cont_flag = true;
					speed = std::min(params_.pMaxRobotSpeed, speed);
					Point2 direction(diff);
					direction.normalize();
					if(robots_[i].StepControl(direction, speed)) {
						std::cerr << "Control incorrect\n";
					}
					robot_global_positions_[i] = robots_[i].GetGlobalCurrentPosition();
				}
				return cont_flag;
			}

			/* auto LloydOffline() { */
			/* 	return LloydOracle(params_.pLloydNumTries, params_.pLloydMaxIterations, num_robots_, world_idf_.GetWorldMap(), params_.pWorldMapSize, params_.pResolution); */
			/* } */

			bool StepOracleN(int const num_steps) {
				bool cont_flag = true;
				for(int i = 0; i < num_steps; ++i) {
					/* std::cout << "StepOracleN: " << i << std::endl; */
					if(not StepOracle()) {
						cont_flag = false;
						break;
					}
				}
				return cont_flag;
			}

			void UpdateOracleMap() {
				for(size_t i = 0; i < num_robots_; ++i) {
					MapUtils::MapBounds index, offset;
					MapUtils::ComputeOffsets(params_.pResolution, robots_[i].GetGlobalCurrentPosition(), params_.pSensorSize, params_.pWorldMapSize, index, offset);
					oracle_map_.block(index.left + offset.left, index.bottom + offset.bottom, offset.width, offset.height) = robots_[i].GetSensorView().block(offset.left, offset.bottom, offset.width, offset.height);
				}
			}

			bool StepOracle() {
				bool cont_flag = true;
				auto voronoi = LloydOffline(params_.pLloydNumTries, params_.pLloydMaxIterations, num_robots_, oracle_map_, params_.pWorldMapSize, params_.pResolution);

				auto voronoi_cells = voronoi.GetVoronoiCells();
#pragma omp parallel for num_threads(num_robots_)
				for(size_t iRobot = 0; iRobot < num_robots_; ++iRobot) {
					for(size_t jCentroid = 0; jCentroid < voronoi_cells.size(); ++jCentroid) {
						cost_matrix_[iRobot][jCentroid] = (robot_global_positions_[iRobot] - voronoi_cells[jCentroid].centroid).norm();
					}
				}
				HungarianAlgorithm HungAlgo;
				std::vector<int> assignment;
				HungAlgo.Solve(cost_matrix_, assignment);

				for(size_t iRobot = 0; iRobot < num_robots_; ++iRobot) {
					cont_flag = false;
					auto goal = voronoi_cells[assignment[iRobot]].centroid;
					if((goal - robot_global_positions_[iRobot]).squaredNorm() > params_.pResolution * params_.pResolution) {
						StepRobotToPoint(iRobot, goal);
						cont_flag = true;
					}
				}
				UpdateOracleMap();
				return cont_flag;
			}

			bool StepDataGenerationLocal(int const steps) {
				bool cont_flag = StepOracleN(steps);
#pragma omp parallel for num_threads(num_robots_)
				for(size_t iRobot = 0; iRobot < num_robots_; ++iRobot) {
					auto robot_local_map = robots_[iRobot].GetRobotLocalMap();
					auto robot_neighbors_pos = GetRobotsInCommunication(iRobot);
					PointVector robot_positions(robot_neighbors_pos.size() + 1);
					Point2 map_translation(params_.pLocalMapSize * params_.pResolution/2., params_.pLocalMapSize * params_.pResolution/2.);
					robot_positions[0] = map_translation;
					int count = 1;
					for(auto const &pos:robot_neighbors_pos) {
						robot_positions[count] = pos + map_translation;
						++count;
					}
					Voronoi voronoi(robot_positions, robot_local_map, params_.pLocalMapSize, params_.pResolution, true, 0);
					/* std::cout << "voronoi computed" << std::endl; */
					voronoi_cells_[iRobot] = voronoi.GetVoronoiCell();
					/* auto centroid = voronoi_cells_[iRobot].centroid; */
					/* std::cout << centroid.x() << " " << centroid.y() << std::endl; */
				}
				return cont_flag;
			}

			auto GetObjectiveValue() {
				ComputeVoronoiCells();
				double obj = 0;
				for(auto const &vcell:voronoi_cells_) {
					obj += vcell.obj;
				}
				return obj;
			}

			auto GetLocalVoronoiFeatures() {

				std::vector <Point3> features(num_robots_);
#pragma omp parallel for num_threads(num_robots_)
				for(size_t iRobot = 0; iRobot < num_robots_; ++iRobot) {
					auto robot_local_map = robots_[iRobot].GetRobotLocalMap();
					auto robot_neighbors_pos = GetRobotsInCommunication(iRobot);
					PointVector robot_positions(robot_neighbors_pos.size() + 1);
					Point2 map_translation(params_.pLocalMapSize * params_.pResolution/2., params_.pLocalMapSize * params_.pResolution/2.);
					robot_positions[0] = map_translation;
					int count = 1;
					for(auto const &pos:robot_neighbors_pos) {
						robot_positions[count] = pos + map_translation;
						++count;
					}
					Voronoi voronoi(robot_positions, robot_local_map, params_.pLocalMapSize, params_.pResolution, true, 0);
					auto vcell = voronoi.GetVoronoiCell();
					Point3 feature(vcell.centroid.x(), vcell.centroid.y(), vcell.mass);
					features[iRobot] = feature;
				}
				return features;
			}

			auto GetVoronoiCells() { return voronoi_cells_; }

			auto GetVoronoiCell(int const robot_id) { return voronoi_cells_[robot_id]; }

			double GetNormalizationFactor() {
				normalization_factor_ = world_idf_.GetNormalizationFactor();
				return normalization_factor_;
			}
	};

} /* namespace CoverageControl */
#endif /*_COVERAGECONTROL_COVERAGE_SYSTEM_H_ */
