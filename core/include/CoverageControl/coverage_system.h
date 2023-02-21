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
#include "plotter.h"
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
			PointVector robot_global_positions_;
			MapType system_map_; // Map with exploration and coverage
			MapType exploration_map_; // Binary map: true for unexplored locations
			MapType explored_idf_map_;
			std::vector <std::list<Point2>> robot_positions_history_;
			double exploration_ratio_ = 0;
			double weighted_exploration_ratio_ = 0;
			double total_idf_weight_ = 0;
			std::vector <PlotterData> plotter_data_;

		public:
			// Initialize IDF with num_gaussians distributions
			// Initialize num_robots with random start positions
			CoverageSystem( Parameters const &params, int const num_gaussians, int const num_robots) : params_{params}, world_idf_{WorldIDF(params_)}{
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
				InitSetup();
			}

			CoverageSystem(Parameters const &params, WorldIDF const &world_idf, std::vector <Point2> const &robot_positions) : params_{params}, world_idf_{WorldIDF(params_)}{
				SetWorldIDF(world_idf);

				// Generate the world map using Cuda
				world_idf_.GenerateMapCuda();
				normalization_factor_ = world_idf_.GetNormalizationFactor();

				robots_.reserve(robot_positions.size());
				for(auto const &pos:robot_positions) {
					robots_.push_back(RobotModel(params_, pos, world_idf_));
				}
				InitSetup();
			}

			CoverageSystem(Parameters const &params, std::vector <BivariateNormalDistribution> const &dists, std::vector <Point2> const &robot_positions) : params_{params}, world_idf_{WorldIDF(params_)}{
				world_idf_.AddNormalDistribution(dists);
				num_robots_ = robot_positions.size();

				// Generate the world map using Cuda
				world_idf_.GenerateMapCuda();
				normalization_factor_ = world_idf_.GetNormalizationFactor();

				robots_.reserve(num_robots_);
				for(auto const &pos:robot_positions) {
					robots_.push_back(RobotModel(params_, pos, world_idf_));
				}
				InitSetup();
			}

			void InitSetup() {
				num_robots_ = robots_.size();
				robot_positions_history_.resize(num_robots_);

				voronoi_cells_.resize(num_robots_);
				communication_maps_.resize(num_robots_);

				robot_global_positions_.resize(num_robots_);
				for(size_t iRobot = 0; iRobot < num_robots_; ++iRobot) {
					robot_global_positions_[iRobot] = robots_[iRobot].GetGlobalCurrentPosition();
				}
				system_map_ = MapType::Constant(params_.pWorldMapSize, params_.pWorldMapSize, 0);
				exploration_map_ = MapType::Constant(params_.pWorldMapSize, params_.pWorldMapSize, 1);
				explored_idf_map_ = MapType::Constant(params_.pWorldMapSize, params_.pWorldMapSize, 0);
				total_idf_weight_ = GetWorldIDF().sum();
				PostStepCommands();
			}

			void UpdateSystemMap() {
				for(size_t i = 0; i < num_robots_; ++i) {
					MapUtils::MapBounds index, offset;
					MapUtils::ComputeOffsets(params_.pResolution, robot_global_positions_[i], params_.pSensorSize, params_.pWorldMapSize, index, offset);
					explored_idf_map_.block(index.left + offset.left, index.bottom + offset.bottom, offset.width, offset.height) = GetRobotSensorView(i).block(offset.left, offset.bottom, offset.width, offset.height);
					exploration_map_.block(index.left + offset.left, index.bottom + offset.bottom, offset.width, offset.height) = MapType::Zero(offset.width, offset.height);
				}
				system_map_ = explored_idf_map_ - exploration_map_;
				exploration_ratio_ = 1.0 - (double)(exploration_map_.sum())/(params_.pWorldMapSize * params_.pWorldMapSize);
				weighted_exploration_ratio_ = (double)(explored_idf_map_.sum())/(total_idf_weight_);
				/* std::cout << "Exploration: " << exploration_ratio_ << " Weighted: " << weighted_exploration_ratio_ << std::endl; */
				/* std::cout << "Diff: " << (exploration_map_.count() - exploration_map_.sum()) << std::endl; */
			}

			inline double GetExplorationRatio() const { return exploration_ratio_; }
			inline double GetWeightedExplorationRatio() const { return weighted_exploration_ratio_; }

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
				world_idf_.GenerateMapCuda();
				normalization_factor_ = world_idf_.GetNormalizationFactor();
			}

			bool StepActions(PointVector const &actions) {
				for(size_t iRobot = 0; iRobot < num_robots_; ++iRobot) {
					Point2 action = actions[iRobot];
					double speed = action.norm();
					Point2 direction = action.normalized();
					if(robots_[iRobot].StepControl(direction, speed)) {
						std::cerr << "Control incorrect\n";
						return 1;
					}
				}
				PostStepCommands();
				return 0;
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

			auto const& GetWorldIDFObject() const { return world_idf_; }
			MapType const& GetWorldIDF() const { return world_idf_.GetWorldMap(); }
			MapType const& GetSystemMap() const { return system_map_; }
			MapType const& GetSystemExplorationMap() const { return exploration_map_; }
			MapType const& GetSystemExploredIDFMap() const { return explored_idf_map_; }

			bool CheckOscillation(size_t const robot_id) {
				auto const &history = robot_positions_history_[robot_id];
				Point2 const last_pos = history.back();
				auto it_end = std::next(history.crbegin(), 6);
				bool flag = false;
				int count = 0;
				std::for_each(history.crbegin(), it_end, [last_pos, &count](Point2 const &pt) { if((pt - last_pos).norm() < kEps) { ++count; }  });
				if(count > 2) { flag = true; }
				return flag;
			}

			void CheckRobotID(size_t const id) const {
				if(id >= num_robots_) {
					throw std::out_of_range{"Robot index more than the number of robots"};
				}
			}

			MapType const& GetRobotLocalMap(size_t const id) {
				CheckRobotID(id);
				return robots_[id].GetRobotLocalMap();
			}

			MapType const& GetRobotMap(size_t const id) {
				CheckRobotID(id);
				return robots_[id].GetRobotMap();
			}

			MapType const& GetRobotExplorationMap(size_t const id) {
				CheckRobotID(id);
				return robots_[id].GetExplorationMap();
			}

			MapType const& GetRobotSystemMap(size_t const id) {
				CheckRobotID(id);
				return robots_[id].GetRobotSystemMap();
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
				std::sort(robot_neighbors_pos.begin(), robot_neighbors_pos.end(), [](Point2 const &a, Point2 const &b) {
						return a.norm() < b.norm();
						});
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
				voronoi_ = Voronoi(robot_global_positions_, GetWorldIDF(), Point2 (params_.pWorldMapSize, params_.pWorldMapSize), params_.pResolution);
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
				return voronoi_.GetSumIDFSiteDistSqr();
			}

			auto GetRobotExplorationFeatures() {
				std::vector <std::vector<double>> features(num_robots_);
#pragma omp parallel for num_threads(num_robots_)
				for(size_t iRobot = 0; iRobot < num_robots_; ++iRobot) {
					features[iRobot] = robots_[iRobot].GetExplorationFeatures();
				}
				return features;
			}

			auto GetRobotVoronoiFeatures() {
				std::vector <std::vector <double> > features(num_robots_);
#pragma omp parallel for num_threads(num_robots_)
				for(size_t iRobot = 0; iRobot < num_robots_; ++iRobot) {
					features[iRobot] = robots_[iRobot].GetVoronoiFeatures();
				}
				return features;
			}

			/* The centroid is computed with orgin of the map, i.e., the lower left corner of the map. */
			/* Uses neighboring robots' positions to compute the centroid. */
			auto GetLocalVoronoiFeatures(int const robot_id) {
				auto const &pos = robot_global_positions_[robot_id];
				MapUtils::MapBounds index, offset;
				MapUtils::ComputeOffsets(params_.pResolution, pos, params_.pLocalMapSize, params_.pWorldMapSize, index, offset);
				auto robot_map = robots_[robot_id].GetRobotMap();
				auto trimmed_local_map = robot_map.block(index.left + offset.left, index.bottom + offset.bottom, offset.width, offset.height);
				Point2 map_size(offset.width, offset.height);

				Point2 map_translation((index.left + offset.left) * params_.pResolution, (index.bottom + offset.bottom) * params_.pResolution);

				auto robot_neighbors_pos = GetRobotsInCommunication(robot_id);
				PointVector robot_positions(robot_neighbors_pos.size() + 1);

				robot_positions[0] = pos - map_translation;
				int count = 1;
				for(auto const &pos:robot_neighbors_pos) {
					robot_positions[count] = pos - map_translation;
					++count;
				}
				Voronoi voronoi(robot_positions, trimmed_local_map, map_size, params_.pResolution, true, 0);
				auto vcell = voronoi.GetVoronoiCell();
				vcell.centroid += Point2(params_.pLocalMapSize / 2., params_.pLocalMapSize / 2.);
				std::vector<double> feature{vcell.centroid.x(), vcell.centroid.y(), vcell.mass, vcell.sum_idf_site_dist_sqr, vcell.sum_idf_goal_dist_sqr, vcell.sum_idf_site_dist, vcell.sum_idf_goal_dist};
				return feature;
			}

			auto GetLocalVoronoiFeatures() {
				std::vector <std::vector<double>> features(num_robots_);
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

			void RenderRecordedMap(std::string const &, std::string const &) const;
			void RecordPlotData(std::vector <int> const &);
			void RecordPlotData() {
				std::vector<int> robot_status(num_robots_, 0);
				RecordPlotData(robot_status);
			}
			void PlotFrontiers(std::string const &, int const &, PointVector const &) const;
			void PlotSystemMap(std::string const &dir_name, int const &step) const {
				std::vector<int> robot_status(num_robots_, 0);
				PlotSystemMap(dir_name, step, robot_status);
			}
			void PlotSystemMap(std::string const &, int const &, std::vector <int> const &) const;
			void PlotMapVoronoi(std::string const &, int const &, Voronoi const &, PointVector const &) const;
			void PlotWorldMap(std::string const &) const;
			void PlotRobotLocalMap(std::string const &, int const &) const;
			void PlotRobotSystemMap(std::string const &, int const &, int const &);
			void PlotRobotIDFMap(std::string const &, int const &, int const &);
			void PlotRobotExplorationMap(std::string const &, int const &, int const &);
			void PlotRobotSensorView(std::string const &, int const &, int const &);
	};

} /* namespace CoverageControl */
#endif /*_COVERAGECONTROL_COVERAGE_SYSTEM_H_ */
