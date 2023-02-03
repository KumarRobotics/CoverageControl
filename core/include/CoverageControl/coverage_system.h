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
#include "world_idf.h"
#include "robot_model.h"
#include "map_utils.h"
#include "voronoi.h"
#include "lloyd_algorithms.h"
#include <lsap/Hungarian.h>

namespace CoverageControl {

	class CoverageSystem {
		private:
			Parameters const params_;
			WorldIDF world_idf_;
			size_t num_robots_ = 0;
			std::vector <RobotModel> robots_;
			std::vector <MapType> communication_maps_;
			double normalization_factor_ = 0;
			Voronoi voronoi_;
			std::vector <VoronoiCell> voronoi_cells_;
			std::random_device rd_;  //Will be used to obtain a seed for the random number engine
			std::mt19937 gen_;
			std::uniform_real_distribution<> distrib_pts_;
			std::vector <std::vector<double>> cost_matrix_;
			PointVector robot_global_positions_;
			MapType system_map_; // Map with exploration and coverage
			MapType exploration_map_; // Binary map: true for unexplored locations
			MapType explored_idf_map_;
			std::vector <std::list<Point2>> robot_positions_history_;
			double exploration_ratio_ = 0;

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
					double sigma(distrib_var(gen_));
					double peak(distrib_peak(gen_));
					BivariateNormalDistribution dist(mean, sigma, peak);
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
				robot_positions_history_.resize(num_robots_);

				cost_matrix_.resize(num_robots_, std::vector<double>(num_robots_));
				voronoi_cells_.resize(num_robots_);
				communication_maps_.resize(num_robots_);

				robot_global_positions_.resize(num_robots_);
				for(size_t iRobot = 0; iRobot < num_robots_; ++iRobot) {
					robot_global_positions_[iRobot] = robots_[iRobot].GetGlobalCurrentPosition();
				}
				system_map_ = MapType::Constant(params_.pWorldMapSize, params_.pWorldMapSize, 0);
				exploration_map_ = MapType::Constant(params_.pWorldMapSize, params_.pWorldMapSize, 1);
				explored_idf_map_ = MapType::Constant(params_.pWorldMapSize, params_.pWorldMapSize, 0);
				PostStepCommands();
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

			void UpdateSystemMap() {
				for(size_t i = 0; i < num_robots_; ++i) {
					MapUtils::MapBounds index, offset;
					MapUtils::ComputeOffsets(params_.pResolution, robot_global_positions_[i], params_.pSensorSize, params_.pWorldMapSize, index, offset);
					explored_idf_map_.block(index.left + offset.left, index.bottom + offset.bottom, offset.width, offset.height) = GetRobotSensorView(i).block(offset.left, offset.bottom, offset.width, offset.height);
					exploration_map_.block(index.left + offset.left, index.bottom + offset.bottom, offset.width, offset.height) = MapType::Zero(params_.pSensorSize, params_.pSensorSize);
				}
				system_map_ = explored_idf_map_ - exploration_map_;
				exploration_ratio_ = 1.0 - (double)(exploration_map_.count())/(params_.pWorldMapSize * params_.pWorldMapSize);
			}

			inline double GetExplorationRatio() const { return exploration_ratio_; }

			void PostStepCommands() {
				UpdateRobotPositions();
				if(params_.pUpdateSystemMap) {
					UpdateSystemMap();
				}
				for(size_t iRobot = 0; iRobot < num_robots_; ++iRobot) {
					auto &history = robot_positions_history_[iRobot];
					if(history.size() > 0 and history.size() == size_t(params_.pRobotPosHistorySize)) {
						history.pop_front();
					} else {
						history.push_back(robot_global_positions_[iRobot]);
					}
				}
			}

			void SetWorldIDF(WorldIDF const &world_idf) { world_idf_ = world_idf;
				normalization_factor_ = world_idf_.GetNormalizationFactor();
			}

			bool StepAction(size_t const robot_id, Point2 const action) {
				double speed = action.norm();
				Point2 direction = action.normalized();
				if(robots_[robot_id].StepControl(direction, speed)) {
					std::cerr << "Control incorrect\n";
					return 1;
				}
				PostStepCommands();
				return 0;
			}

			bool StepControl(size_t robot_id, Point2 const &direction, double const speed) {
				if(robots_[robot_id].StepControl(direction, speed)) {
					std::cerr << "Control incorrect\n";
					return 1;
				}
				PostStepCommands();
				return 0;
			}

			// Sets positions of all robots with respect to the local start position
			void SetRobotPositions(std::vector<Point2> const &positions) {
				if(positions.size() != num_robots_) {
					throw std::length_error{"The size of the positions don't match with the number of robots"};
				}
				for(size_t i = 0; i < num_robots_; ++i) {
					robots_[i].SetRobotPosition(positions[i]);
				}
				std::cout << "Robot positions updated\n";
				PostStepCommands();
			}

			void UpdateRobotPositions() {
				for(size_t iRobot = 0; iRobot < num_robots_; ++iRobot) {
					robot_global_positions_[iRobot] = robots_[iRobot].GetGlobalCurrentPosition();
				}
			}

			PointVector GetRobotPositions() {
				UpdateRobotPositions();
				return robot_global_positions_;
			}

			Point2 GetRobotPosition(int const robot_id) const { return robots_[robot_id].GetGlobalCurrentPosition(); }

			MapType const& GetWorldIDF() const { return world_idf_.GetWorldMap(); }
			MapType const& GetSystemMap() const { return system_map_; }
			MapType const& GetSystemExplorationMap() const { return exploration_map_; }
			MapType const& GetSystemExploredIDFMap() const { return explored_idf_map_; }

			void CheckRobotID(size_t const id) const {
				if(id >= num_robots_) {
					throw std::out_of_range{"Robot index more than the number of robots"};
				}
			}

			MapType const& GetRobotLocalMap(size_t const id) {
				CheckRobotID(id);
				return robots_[id].GetRobotLocalMap();
			}

			MapTypeBool const& GetRobotExplorationMap(size_t const id) {
				CheckRobotID(id);
				return robots_[id].GetExplorationMap();
			}

			MapType const& GetRobotObstacleMap(size_t const id) {
				CheckRobotID(id);
				return robots_[id].GetObstacleMap();
			}

			MapType const& GetRobotSensorView(size_t const id) {
				CheckRobotID(id);
				return robots_[id].GetSensorView();
			}

			// Get robots within a square communication range
			auto GetRobotsInCommunication(size_t const id) const {
				CheckRobotID(id);
				PointVector robot_neighbors_pos;
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
				}
				return robot_neighbors_pos;
			}

			MapType const& GetCommunicationMap(size_t const id) {
				communication_maps_[id] = MapType::Zero(params_.pLocalMapSize, params_.pLocalMapSize);
				auto robot_neighbors_pos = GetRobotsInCommunication(id);
				double comm_scale = (params_.pCommunicationRange * 2.) / params_.pLocalMapSize;
				Point2 map_translation(params_.pLocalMapSize * comm_scale * params_.pResolution/2., params_.pLocalMapSize * comm_scale * params_.pResolution/2.);
				for(Point2 const& relative_pos:robot_neighbors_pos) {
					Point2 map_pos = relative_pos + map_translation;
					int pos_idx, pos_idy;
					MapUtils::GetClosestGridCoordinate(params_.pResolution * comm_scale, map_pos, pos_idx, pos_idy);
					if(pos_idx < params_.pLocalMapSize and pos_idy < params_.pLocalMapSize and pos_idx >= 0 and pos_idy >= 0) {
						communication_maps_[id](pos_idx, pos_idy) = 1;
					}
				}
				return communication_maps_[id];
			}

			void ComputeVoronoiCells() {
				UpdateRobotPositions();
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
				if(robots_[robot_id].StepControl(direction, speed)) {
					std::cerr << "Control incorrect\n";
					return 1;
				}
				PostStepCommands();
				return 0;
			}

			bool StepRobotsToGoals(PointVector const &goals, PointVector &actions) {
				bool cont_flag = false;
				UpdateRobotPositions();
/* #pragma omp parallel for num_threads(num_robots_) */
				for(size_t iRobot = 0; iRobot < num_robots_; ++iRobot) {
					actions[iRobot] = Point2(0, 0);
					Point2 diff = goals[iRobot] - robot_global_positions_[iRobot];
					double dist = diff.norm();
					double speed = dist / params_.pTimeStep;
					if(speed <= kEps) {
						continue;
					}
					speed = std::min(params_.pMaxRobotSpeed, speed);
					Point2 direction(diff);
					direction.normalize();
					actions[iRobot] = speed * direction;
					if(StepControl(iRobot, direction, speed)) {
						std::cerr << "Control incorrect\n";
					}
					cont_flag = true;
				}
				PostStepCommands();
				return cont_flag;
			}

			auto GetObjectiveValue() {
				ComputeVoronoiCells();
				return voronoi_.GetObjValue();
			}

			/* The centroid is computed with orgin of the map, i.e., the lower left corner of the map. */
			auto GetLocalVoronoiFeatures(int const robot_id) { 
					auto robot_local_map = robots_[robot_id].GetRobotLocalMap();
					auto robot_neighbors_pos = GetRobotsInCommunication(robot_id);
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
					/* vcell.centroid -= map_translation; */
					Point3 feature(vcell.centroid.x(), vcell.centroid.y(), vcell.mass);
					return feature;
			}

			auto GetLocalVoronoiFeatures() {
				std::vector <Point3> features(num_robots_);
#pragma omp parallel for num_threads(num_robots_)
				for(size_t iRobot = 0; iRobot < num_robots_; ++iRobot) {
					features[iRobot] = GetLocalVoronoiFeatures(iRobot);
				}
				return features;
			}

			auto GetVoronoiCells() { return voronoi_cells_; }

			auto GetVoronoiCell(int const robot_id) { return voronoi_cells_[robot_id]; }

			double GetNormalizationFactor() {
				normalization_factor_ = world_idf_.GetNormalizationFactor();
				return normalization_factor_;
			}

			inline int WriteRobotPositions(std::string const &file_name) const {
				std::ofstream file_obj(file_name);
				if(!file_obj) {
					std::cerr << "[Error] Could not open " << file_name << " for writing." << std::endl;
					return 1;
				}
				for(auto const &pos:robot_global_positions_) {
					file_obj << pos[0] << " " << pos[1] << std::endl;
				}
				file_obj.close();
				return 0;
			}

			inline int WriteRobotPositions(std::string const &file_name, PointVector const &positions) {
				std::ofstream file_obj(file_name);
				if(!file_obj) {
					std::cerr << "[Error] Could not open " << file_name << " for writing." << std::endl;
					return 1;
				}
				for(auto const &pos:positions) {
					file_obj << pos[0] << " " << pos[1] << std::endl;
				}
				file_obj.close();
				return 0;
			}

			void PlotFrontiers(std::string const &, int const &, PointVector const &) const;
			void PlotSystemMap(std::string const &dir_name, int const &step) const {
				std::vector<int> robot_status(num_robots_, 0);
				PlotSystemMap(dir_name, step, robot_status);
			}
			void PlotSystemMap(std::string const &, int const &, std::vector <int> const &) const ;
			void PlotMapVoronoi(std::string const &, int const &, Voronoi const &, PointVector const &) const ;
			void PlotWorldMap(std::string const &dir_name) const ;
	};

} /* namespace CoverageControl */
#endif /*_COVERAGECONTROL_COVERAGE_SYSTEM_H_ */
