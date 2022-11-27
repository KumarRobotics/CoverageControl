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

namespace CoverageControl {

	class RobotModel {

		private:

			Parameters const params_;

			Point2 global_start_position_, global_current_position_;
			Point2 local_start_position_, local_current_position_;
			std::vector <Point2> robot_positions_; // Stores the local positions of the robot in order
			double normalization_factor_ = 0;

			MapType robot_map_; // Stores what the robot has seen. Has the same reference as world map.
			MapType sensor_view_; // Stores the current sensor view of the robot
			MapType local_map_; // Stores the local map of the robot
			std::shared_ptr <const WorldIDF> world_idf_; // Robots cannot change the world

			// Gets the sensor data from world IDF at the global_current_position_ and updates robot_map_
			void UpdateRobotMap() {
				sensor_view_ = MapType::Zero(params_.pSensorSize, params_.pSensorSize);
				if(MapUtils::IsPointOutsideBoundary(params_.pResolution, global_current_position_, params_.pSensorSize, params_.pWorldMapSize)) {
					return;
				}
				world_idf_->GetSubWorldMap(global_current_position_, params_.pSensorSize, sensor_view_);
				MapUtils::MapBounds index, offset;
				MapUtils::ComputeOffsets(params_.pResolution, global_current_position_, params_.pSensorSize, params_.pWorldMapSize, index, offset);
				robot_map_.block(index.left + offset.left, index.bottom + offset.bottom, offset.width, offset.height) = sensor_view_.block(offset.left, offset.bottom, offset.width, offset.height);
			}

		public:

			// Constructor: requires global_start_position_ and world_idf_
			RobotModel (Parameters const &params, Point2 const &global_start_position, WorldIDF const &world_idf): params_{params}, global_start_position_{global_start_position} {
				world_idf_ = std::make_shared<const WorldIDF> (world_idf);
				normalization_factor_ = world_idf_->GetNormalizationFactor();
				global_current_position_ = global_start_position_;

				sensor_view_ = MapType::Zero(params_.pSensorSize, params_.pSensorSize);
				local_map_ = MapType::Zero(params_.pLocalMapSize, params_.pLocalMapSize);
				local_start_position_ = Point2{0,0};
				robot_positions_.reserve(params_.pEpisodeSteps); // More steps can be executed. But reserving for efficiency
				local_current_position_ = local_start_position_;
				robot_positions_.push_back(local_current_position_);
				robot_map_ = MapType::Constant(params_.pRobotMapSize, params_.pRobotMapSize, params_.pUnknownImportance * normalization_factor_);
				UpdateRobotMap();
			}

			// Time step robot with the given control direction and speed
			// Direction cannot be zero-vector is speed is not zero
			// speed needs to be positive
			int StepControl(Point2 const &direction, double const &speed) {
				auto dir = direction;
				auto sp = speed;
				Point2 new_pos(0,0);
				if(sp > params_.pMaxRobotSpeed) { sp = params_.pMaxRobotSpeed; }
				if(sp < 0) {
					std::cerr << "Speed needs to be non-negative\n";
					return 1;
				}
				if(dir.Normalize() and sp > kEps) {
					std::cerr << "Zero-vector direction given in control\n";
					return 1;
				}
				if(sp > kEps) {
					new_pos = local_current_position_ + dir * sp * params_.pTimeStep;
				}
				UpdateRobotPosition(new_pos);
				return 0;
			}

			void UpdateRobotPosition(Point2 const &new_pos) {
				local_current_position_ = new_pos;
				robot_positions_.push_back(local_current_position_);
				global_current_position_ = local_current_position_ + global_start_position_;
				UpdateRobotMap();
			}

			Point2 GetGlobalStartPosition() const { return global_start_position_; }
			Point2 GetGlobalCurrentPosition() const { return global_current_position_; }
			std::vector <Point2> GetAllPositions() const { return robot_positions_; }

			const MapType& GetRobotMap() const { return robot_map_; }
			const MapType& GetSensorView() const { return sensor_view_; }

			const MapType& GetRobotLocalMap() {
				local_map_ = MapType::Zero(params_.pLocalMapSize, params_.pLocalMapSize);
				if(not MapUtils::IsPointOutsideBoundary(params_.pResolution, global_current_position_, params_.pLocalMapSize, params_.pWorldMapSize)) {
					MapUtils::GetSubMap(params_.pResolution, global_current_position_, params_.pLocalMapSize, params_.pRobotMapSize, robot_map_, local_map_);
				}
				return local_map_;
			}

	};

} /* namespace CoverageControl */
#endif /* COVERAGECONTROL_ROBOTMODEL_H_ */

