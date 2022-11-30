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
			Voronoi voronoi_;
			double normalization_factor_ = 0;
			std::vector <VoronoiCell> voronoi_cells_;
			std::random_device rd_;  //Will be used to obtain a seed for the random number engine
			std::mt19937 gen_;
			std::uniform_real_distribution<> distrib_pts_;
			std::vector <std::vector<double>> cost_matrix_;

		public:
			// Initialize IDF with num_gaussians distributions
			// Initialize num_robots with random start positions
			CoverageSystem(Parameters const &params, int const num_gaussians, int const num_robots) : params_{params}, world_idf_{WorldIDF(params_)}{
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
				world_idf_.GenerateMapCuda();
				normalization_factor_ = world_idf_.GetNormalizationFactor();

				robots_.reserve(num_robots);
				for(int i = 0; i < num_robots; ++i) {
					Point2 start_pos(distrib_pts_(gen_), distrib_pts_(gen_));
					robots_.push_back(RobotModel(params_, start_pos, world_idf_));
				}
				num_robots_ = robots_.size();
				oracle_map_ = MapType::Constant(params_.pWorldMapSize, params_.pWorldMapSize, params_.pUnknownImportance * params_.pNorm);
				cost_matrix_.resize(num_robots_, std::vector<double>(num_robots_));
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

			const MapType& GetOracleMap() {
				return oracle_map_;
			}
			const MapType& GetRobotSensorView(size_t const id) {
				if(id < num_robots_) {
					return robots_[id].GetSensorView();
				} else {
					throw std::out_of_range{"Robot index more than the number of robots"};
				}
			}

			auto GetRobotsInCommunication(size_t const id) {
				PointVector robot_neighbors_pos;
				if(id < num_robots_) {
					double comm_range_sqr = params_.pCommunicationRange * params_.pCommunicationRange;
					auto robot_positions = GetRobotPositions();
					for(size_t i = 0; i < num_robots_; ++i) {
						if(id == i) {
							continue;
						}
						auto relative_pos = robot_positions[i] - robot_positions[id];
						if(relative_pos.NormSqr() <= comm_range_sqr) {
							robot_neighbors_pos.push_back(relative_pos);
						}
					}
				} else {
					throw std::out_of_range{"Robot index more than the number of robots"};
				}
				return robot_neighbors_pos;

			}

			const MapType& GetCommunicationMap(size_t const id) {
				communication_map_ = MapType::Zero(params_.pLocalMapSize, params_.pLocalMapSize);
				auto robot_neighbors_pos = GetRobotsInCommunication(id);
				for(auto const &relative_pos:robot_neighbors_pos) {
					int pos_idx, pos_idy;
					MapUtils::GetClosestGridCoordinate(params_.pResolution, relative_pos, pos_idx, pos_idy);
					communication_map_(pos_idx, pos_idy) = 1;
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

			bool StepRobotToPoint(int const robot_id, Point2 const &goal, double const speed_factor = 1) {
					auto diff = goal - robots_[robot_id].GetGlobalCurrentPosition();
					auto dist = diff.Norm();
					double speed = speed_factor * dist / params_.pTimeStep;
					if(speed <= kEps) {
						return 0;
					}
					speed = std::min(params_.pMaxRobotSpeed, speed);
					auto direction = diff; direction.Normalize();
					if(robots_[robot_id].StepControl(direction, speed)) {
						std::cerr << "Control incorrect\n";
						return 1;
					}
					return 0;
			}

			bool StepLloyd() {
				bool cont_flag = false;
				ComputeVoronoiCells();
				for(size_t i = 0; i < num_robots_; ++i) {
					auto diff = voronoi_cells_[i].centroid - voronoi_cells_[i].site;
					auto dist = diff.Norm();
					double speed = 2 * voronoi_cells_[i].mass * dist;
					/* double speed = diff.Norm() / params_.pTimeStep; */
					if(dist <= params_.pResolution) {
						speed = dist / params_.pTimeStep;
					}
					if(speed <= kEps) {
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
					cont_flag = StepLloyd();
				}
			}

			auto LloydOracle1(int const num_tries, int const max_iterations, int num_sites, MapType const &map, int const map_size, double const res) {
				auto sites = GetRobotPositions();
				bool cont_flag = true;
				Voronoi voronoi(sites, map, map_size, res);
				std::vector<VoronoiCell> voronoi_cells = voronoi.GetVoronoiCells();
				int iSteps = 0;
				for(iSteps = 0; iSteps < max_iterations and cont_flag == true; ++iSteps) {
					cont_flag = false;
					voronoi_cells = voronoi.GetVoronoiCells();
					for(int iSite = 0; iSite < num_sites; ++iSite) {
						auto diff = voronoi_cells[iSite].centroid - voronoi_cells[iSite].site;
						if(diff.Norm() < res) {
							continue;
						}
						cont_flag = true;
						sites[iSite] = voronoi_cells[iSite].centroid;
					}
					voronoi.UpdateSites(sites);
				}
				std::cout << "No. of voronoi steps: " << iSteps << std::endl;
				return voronoi_cells;
			}
			auto LloydOracle(int const num_tries, int const max_iterations, int num_sites, MapType const &map, int const map_size, double const res) {
				std::vector <std::vector<VoronoiCell>> all_voronoi_cells;
				all_voronoi_cells.resize(num_tries, std::vector<VoronoiCell>(num_sites));
				std::vector <double> obj_values;
				obj_values.resize(num_tries, 0);
				std::uniform_real_distribution<> distrib_pts(0, map_size * res);

				/* #pragma omp parallel for */
				for(int iter = 0; iter < num_tries; ++iter) {
					std::vector <Point2> sites;
					sites.resize(num_sites);
					for(int iSite = 0; iSite < num_sites; ++iSite) {
						sites[iSite] = Point2(distrib_pts(gen_), distrib_pts(gen_));
					}
					bool cont_flag = true;
					/* std::cout << "voronoi start" << std::endl; */
					Voronoi voronoi(sites, map, map_size, res);
					/* std::cout << "voronoi end" << std::endl; */
					std::vector<VoronoiCell> voronoi_cells = voronoi.GetVoronoiCells();
					int iSteps = 0;
					for(iSteps = 0; iSteps < max_iterations and cont_flag == true; ++iSteps) {
						cont_flag = false;
						voronoi_cells = voronoi.GetVoronoiCells();
						for(int iSite = 0; iSite < num_sites; ++iSite) {
							auto diff = voronoi_cells[iSite].centroid - voronoi_cells[iSite].site;
							if(diff.Norm() < res) {
								continue;
							}
							cont_flag = true;
							sites[iSite] = voronoi_cells[iSite].centroid;
						}
						voronoi.UpdateSites(sites);
					}
					std::cout << "No. of voronoi steps: " << iSteps << std::endl;
					all_voronoi_cells[iter] = voronoi_cells;
					obj_values[iter] = voronoi.GetObjValue();
				}
				int best_vornoi_idx = 0;
				double min = obj_values[0];
				for(int iter = 1; iter < num_tries; ++iter) {
					if(obj_values[iter] < min) {
						min = obj_values[iter];
						best_vornoi_idx = iter;
					}
				}
				return all_voronoi_cells[best_vornoi_idx];
			}

			auto LloydOffline() {
				return LloydOracle(params_.pLloydNumTries, params_.pLloydMaxIterations, num_robots_, world_idf_.GetWorldMap(), params_.pWorldMapSize, params_.pResolution);
			}

			bool StepOracleN(int num_steps) {
				bool cont_flag = true;
				for(int i = 0; i < num_steps; ++i) {
					std::cout << "StepOracleN: " << i << std::endl;
					if(not StepOracle()) {
						cont_flag = false;
						break;
					}
				}
				return cont_flag;
			}

			bool StepOracle() {
				bool cont_flag = true;
				auto robot_positions = GetRobotPositions();
				for(size_t i = 0; i < num_robots_; ++i) {
					MapUtils::MapBounds index, offset;
					MapUtils::ComputeOffsets(params_.pResolution, robot_positions[i], params_.pSensorSize, params_.pWorldMapSize, index, offset);
					oracle_map_.block(index.left + offset.left, index.bottom + offset.bottom, offset.width, offset.height) = robots_[i].GetSensorView().block(offset.left, offset.bottom, offset.width, offset.height);
				}

				auto voronoi_cells = LloydOracle(params_.pLloydNumTries, params_.pLloydMaxIterations, num_robots_, oracle_map_, params_.pWorldMapSize, params_.pResolution);

				for(size_t iRobot = 0; iRobot < num_robots_; ++iRobot) {
					for(size_t jCentroid = 0; jCentroid < voronoi_cells.size(); ++jCentroid) {
						cost_matrix_[iRobot][jCentroid] = (robot_positions[iRobot] - voronoi_cells[jCentroid].centroid).Norm();
					}
				}
				HungarianAlgorithm HungAlgo;
				std::vector<int> assignment;
				HungAlgo.Solve(cost_matrix_, assignment);

				for(size_t iRobot = 0; iRobot < num_robots_; ++iRobot) {
					StepRobotToPoint(iRobot, voronoi_cells[assignment[iRobot]].centroid);
				}
				return cont_flag;
			}

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
