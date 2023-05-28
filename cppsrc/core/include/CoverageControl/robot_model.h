/**
 * Class file for robot model
 * Makes heavy use of paramters.h, so please ensure that the parameters are set appropriately.
 * The names of the parameters start with lower-case p and use CamelCase
 **/

#ifndef COVERAGECONTROL_ROBOTMODEL_H_
#define COVERAGECONTROL_ROBOTMODEL_H_

#include <cmath>
#include <iostream>
#include <memory>

#include "constants.h"
#include "parameters.h"
#include "typedefs.h"
#include "world_idf.h"
#include "map_utils.h"
#include "voronoi.h"

namespace CoverageControl {

	class RobotModel {

		private:

			Parameters const params_;

			Point2 global_start_position_, global_current_position_;
			Point2 local_start_position_, local_current_position_;
			double normalization_factor_ = 0;

			MapType robot_map_; // Stores what the robot has seen. Has the same reference as world map.
			MapType sensor_view_; // Stores the current sensor view of the robot
			MapType local_map_; // Stores the local map of the robot
			MapType obstacle_map_; // Stores the obstacle map
			MapType system_map_; // Stores the obstacle map
			MapType local_exploration_map_; // Binary map: true for unexplored locations
			MapType exploration_map_; // Binary map: true for unexplored locations
			std::shared_ptr <const WorldIDF> world_idf_; // Robots cannot change the world
			double time_step_dist_ = 0;
			double sensor_area_ = 0;

			// Gets the sensor data from world IDF at the global_current_position_ and updates robot_map_
			void UpdateSensorView() {
				sensor_view_ = MapType::Zero(params_.pSensorSize, params_.pSensorSize);
				if(MapUtils::IsPointOutsideBoundary(params_.pResolution, global_current_position_, params_.pSensorSize, params_.pWorldMapSize)) {
					return;
				}
				world_idf_->GetSubWorldMap(global_current_position_, params_.pSensorSize, sensor_view_);
			}

			void UpdateRobotMap() {
				MapUtils::MapBounds index, offset;
				MapUtils::ComputeOffsets(params_.pResolution, global_current_position_, params_.pSensorSize, params_.pWorldMapSize, index, offset);
				robot_map_.block(index.left + offset.left, index.bottom + offset.bottom, offset.width, offset.height) = sensor_view_.block(offset.left, offset.bottom, offset.width, offset.height);
			}

			void UpdateExplorationMap() {
				if(MapUtils::IsPointOutsideBoundary(params_.pResolution, global_current_position_, params_.pSensorSize, params_.pWorldMapSize)) {
					return;
				}
				MapUtils::MapBounds index, offset;
				MapUtils::ComputeOffsets(params_.pResolution, global_current_position_, params_.pSensorSize, params_.pWorldMapSize, index, offset);
				exploration_map_.block(index.left + offset.left, index.bottom + offset.bottom, offset.width, offset.height) = MapType::Zero(offset.width, offset.height);
			}

		public:

			// Constructor: requires global_start_position_ and world_idf_
			RobotModel (Parameters const &params, Point2 const &global_start_position, WorldIDF const &world_idf): params_{params}, global_start_position_{global_start_position} {
				world_idf_ = std::make_shared<const WorldIDF> (world_idf);
				normalization_factor_ = world_idf_->GetNormalizationFactor();
				global_current_position_ = global_start_position_;

				sensor_view_ = MapType::Zero(params_.pSensorSize, params_.pSensorSize);
				local_map_ = MapType::Zero(params_.pLocalMapSize, params_.pLocalMapSize);
				obstacle_map_ = MapType::Zero(params_.pLocalMapSize, params_.pLocalMapSize);
				if(params_.pRobotMapUseUnknownImportance == true) {
					robot_map_ = MapType::Constant(params_.pRobotMapSize, params_.pRobotMapSize, params_.pUnknownImportance * params_.pNorm);
				} else {
					robot_map_ = MapType::Zero(params_.pRobotMapSize, params_.pRobotMapSize);
				}

				local_start_position_ = Point2{0,0};
				local_current_position_ = local_start_position_;

				if(params_.pUpdateExplorationMap == true) {
					exploration_map_ = MapType::Constant(params_.pRobotMapSize, params_.pRobotMapSize, 1);
					local_exploration_map_ = MapType::Constant(params_.pRobotMapSize, params_.pRobotMapSize, 1);
					UpdateExplorationMap();
				} else {
					exploration_map_ = MapType::Constant(params_.pRobotMapSize, params_.pRobotMapSize, 0);
					local_exploration_map_ = MapType::Constant(params_.pRobotMapSize, params_.pRobotMapSize, 0);
				}

				if(params_.pUpdateSensorView == true) {
					UpdateSensorView();
				}
				if(params_.pUpdateRobotMap == false) {
					robot_map_ = world_idf_->GetWorldMap();
				} else {
					UpdateRobotMap();
				}

				time_step_dist_ = params_.pMaxRobotSpeed * params_.pTimeStep * params_.pResolution;
				sensor_area_ = params_.pSensorSize * params_.pSensorSize;

			}

			// Time step robot with the given control direction and speed
			// Direction cannot be zero-vector is speed is not zero
			// speed needs to be positive
			int StepControl(Point2 const &direction, double const &speed) {
				auto dir = direction;
				auto sp = speed;
				Point2 new_pos = local_current_position_;
				if(sp > params_.pMaxRobotSpeed) { sp = params_.pMaxRobotSpeed; }
				if(sp < 0 or (dir.norm() < kEps and sp >= kEps)) {
					std::cout << sp << " " << dir.norm() << std::endl;
					std::cerr << "Speed needs to be non-negative\n";
					std::cerr << "Zero-vector direction cannot be given in control\n";
					return 1;
				}
				/* if(dir.normalize() and sp > kEps) { */
				/* 	std::cerr << "Zero-vector direction given in control\n"; */
				/* 	return 1; */
				/* } */
				dir.normalize();
				if(sp > kEps) {
					new_pos = local_current_position_ + dir * sp * params_.pTimeStep;
				}
				SetRobotPosition(new_pos);
				return 0;
			}

			void SetRobotPosition(Point2 const &pos) {
				Point2 new_global_pos = pos + global_start_position_;
				double eps = 0.0001;
				if(new_global_pos.x() <= 0) { new_global_pos[0] = 0 + eps; }
				if(new_global_pos.y() <= 0) { new_global_pos[1] = 0 + eps; }
				double max_xy = params_.pWorldMapSize * params_.pResolution;
				if(new_global_pos.x() >= max_xy) { new_global_pos[0] = max_xy - eps; }
				if(new_global_pos.y() >= max_xy) { new_global_pos[1] = max_xy - eps; }

				local_current_position_ = new_global_pos - global_start_position_;
				global_current_position_ = new_global_pos;
				if(params_.pUpdateSensorView == true) {
					UpdateSensorView();
				}
				if(params_.pUpdateRobotMap == true) {
					UpdateRobotMap();
				}
				if(params_.pUpdateExplorationMap == true) {
					UpdateExplorationMap();
				}
			}

			Point2 GetGlobalStartPosition() const { return global_start_position_; }
			Point2 GetGlobalCurrentPosition() const { return global_current_position_; }

			const MapType& GetRobotMap() const { return robot_map_; }
			const MapType& GetSensorView() const { return sensor_view_; }

			const MapType& GetRobotSystemMap() {
				GetRobotLocalMap();
				GetExplorationMap();
				system_map_ = local_map_ - local_exploration_map_;
				return system_map_;
			}

			const MapType& GetRobotLocalMap() {
				/* local_map_ = MapType::Constant(params_.pLocalMapSize, params_.pLocalMapSize, -1.0); */
				local_map_ = MapType::Constant(params_.pLocalMapSize, params_.pLocalMapSize, 0.);
				if(not MapUtils::IsPointOutsideBoundary(params_.pResolution, global_current_position_, params_.pLocalMapSize, params_.pWorldMapSize)) {
					MapUtils::GetSubMap(params_.pResolution, global_current_position_, params_.pRobotMapSize, robot_map_, params_.pLocalMapSize, local_map_);
				}
				return local_map_;
			}

			const MapType& GetExplorationMap() {
				/* local_exploration_map_ = MapType::Constant(params_.pLocalMapSize, params_.pLocalMapSize, 0); */
				local_exploration_map_ = MapType::Constant(params_.pLocalMapSize, params_.pLocalMapSize, -1.0);
				if(not MapUtils::IsPointOutsideBoundary(params_.pResolution, global_current_position_, params_.pLocalMapSize, params_.pWorldMapSize)) {
					MapUtils::GetSubMap(params_.pResolution, global_current_position_, params_.pRobotMapSize, exploration_map_, params_.pLocalMapSize, local_exploration_map_);
				}
				return local_exploration_map_;
			}

			const MapType& GetObstacleMap() {
				obstacle_map_ = MapType::Ones(params_.pLocalMapSize, params_.pLocalMapSize);
				MapUtils::MapBounds index, offset;
				ComputeOffsets(params_.pResolution, global_current_position_, params_.pLocalMapSize, params_.pRobotMapSize, index, offset);
				obstacle_map_.block(offset.left, offset.bottom, offset.width, offset.height) = MapType::Zero(offset.width, offset.height);
				return obstacle_map_;
			}

			auto GetExplorationFeatures() {
				double eps = 0.0001;
				queue_t frontiers;
				Point2 const &pos = global_current_position_;

				double factor = 4;
				double pos_x_lim = std::min(pos[0] + factor * time_step_dist_, double(params_.pWorldMapSize));
				double pos_y_lim = std::min(pos[1] + factor * time_step_dist_, double(params_.pWorldMapSize));
				for(double pos_x = std::max(pos[0] - factor * time_step_dist_, eps); pos_x < pos_x_lim;  pos_x += 1 * params_.pResolution ) {
					for(double pos_y = std::max(pos[1] - factor * time_step_dist_, eps); pos_y < pos_y_lim; pos_y += 1 * params_.pResolution ) {
						Point2 qpos{ pos_x, pos_y };
						double dist = (qpos - pos).norm();
						dist = std::max(dist, time_step_dist_);
						MapUtils::MapBounds index, offset;
						MapUtils::ComputeOffsets(params_.pResolution, qpos, params_.pSensorSize, params_.pWorldMapSize, index, offset);
						double unexplored = exploration_map_.block(index.left + offset.left, index.bottom + offset.bottom, offset.width, offset.height).sum();
						double benefit = unexplored;
						if(unexplored > 0.1 * sensor_area_) {
							double idf_value = robot_map_.block(index.left + offset.left, index.bottom + offset.bottom, offset.width, offset.height).count();
							benefit += 2 * idf_value;
						}
						double bcr = benefit/dist;

						if(frontiers.size() < size_t(params_.pNumFrontiers)) {
							frontiers.push(Frontier(qpos, bcr));
						} else {
							auto worst_frontier = frontiers.top();
							if(worst_frontier.value < bcr) {
								frontiers.pop();
								frontiers.push(Frontier(qpos, bcr));
							}
						}
					}
				}
				std::vector <double> frontier_features(params_.pNumFrontiers * 2);
				int count = 0;
				while(!frontiers.empty()) {
					auto point = frontiers.top();
					frontier_features[count++] = point.pt[0] - pos[0];
					frontier_features[count++] = point.pt[1] - pos[1];
					frontiers.pop();
				}
				return frontier_features;
			}

			/* The centroid is computed with orgin of the map, i.e., the lower left corner of the map. */
			/* Only the local map is considered for the computation of the centroid. */
			auto GetVoronoiFeatures() { 
				auto const &pos = global_current_position_;
				MapUtils::MapBounds index, offset;
				MapUtils::ComputeOffsets(params_.pResolution, pos, params_.pLocalMapSize, params_.pWorldMapSize, index, offset);
				auto trimmed_local_map = robot_map_.block(index.left + offset.left, index.bottom + offset.bottom, offset.width, offset.height);

				Point2 map_size(offset.width, offset.height);
				Point2 map_translation((index.left + offset.left) * params_.pResolution, (index.bottom + offset.bottom) * params_.pResolution);

				PointVector robot_positions(1);
				robot_positions[0] = pos - map_translation;
				Voronoi voronoi(robot_positions, trimmed_local_map, map_size, params_.pResolution, true, 0);
				auto vcell = voronoi.GetVoronoiCell();
				/* vcell.centroid += Point2(params_.pLocalMapSize / 2., params_.pLocalMapSize / 2.); */
				return vcell.GetFeatureVector();
			}

	};

} /* namespace CoverageControl */
#endif /* COVERAGECONTROL_ROBOTMODEL_H_ */
