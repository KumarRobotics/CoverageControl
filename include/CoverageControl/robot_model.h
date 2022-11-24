/**
 * Class file for robot model
 * Makes heavy use of paramters.h, so please ensure that the parameters are set appropriately.
 * The names of the parameters start with lower-case p and use CamelCase
 **/

#ifndef COVERAGECONTROL_ROBOTMODEL_H_
#define COVERAGECONTROL_ROBOTMODEL_H_

#include <cmath>

#include "constants.h"
#include "parameters.h"
#include "typedefs.h"
#include "world_idf.h"
#include "map_utils.h"

namespace CoverageControl {

	class RobotModel {

		private:

			Point2 global_start_position_, global_current_position_;
			Point2 local_start_position_, local_current_position_;
			std::vector <Point2> robot_positions_; // Stores the local positions of the robot in order


			MapType robot_map_; // Stores what the robot has seen. Has the same reference as world map.
			MapType sensor_view_; // Stores the current sensor view of the robot
			WorldIDF const &world_idf_; // Robots cannot change the world

			// Gets the sensor data from world IDF at the global_current_position_ and updates robot_map_
			void UpdateRobotMap() {
				sensor_view_ = MapType::Zero(pSensorSize, pSensorSize);
				if(MapUtils::IsPointOutsideBoundary(global_current_position_, pSensorSize, pWorldMapSize)) {
					return;
				}
				world_idf_.GetSubWorldMap(global_current_position_, pSensorSize, sensor_view_);
				MapUtils::MapBounds index, offset;
				MapUtils::ComputeOffsets(global_current_position_, pSensorSize, pWorldMapSize, index, offset);
				robot_map_.block(index.left + offset.left, index.bottom + offset.bottom, offset.width, offset.height) = sensor_view_.block(offset.left, offset.bottom, offset.width, offset.height);
			}

		public:

			// Constructor: requires global_start_position_ and world_idf_
			RobotModel (Point2 const &global_start_position, WorldIDF const &world_idf):
				global_start_position_{global_start_position},
				world_idf_{world_idf} {

				local_start_position_ = Point2{0,0};
				robot_positions_.reserve(pEpisodeSteps); // More steps can be executed. But reserving for efficiency
				local_current_position_ = local_start_position_;
				robot_positions_.push_back(local_current_position_);
				robot_map_ = MapType::Constant(pRobotMapSize, pRobotMapSize, pUnknownImportance);
				UpdateRobotMap();
			}

			// Time step robot with the given control direction and speed
			// Direction cannot be zero-vector is speed is not zero
			// speed needs to be positive
			int StepControl(Point2 direction, double speed) {
				Point2 new_pos{0,0};
				if(speed > pMaxRobotSpeed) { speed = pMaxRobotSpeed; }
				if(speed < 0) {
					std::cerr << "Speed needs to be non-negative\n";
					return 1;
				}
				if(direction.Normalize() and speed > kEps) {
					std::cerr << "Zero-vector direction given in control\n";
					return 1;
				}
				if(speed > kEps) {
					new_pos = local_current_position_ + direction * speed * pTimeStep;
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

			Point2 GetGlobalStartPosition() { return global_start_position_; }
			Point2 GetGlobalCurrentPosition() { return global_current_position_; }
			std::vector <Point2> GetAllPositions() { return robot_positions_; }

			MapType GetRobotMap() { return robot_map_; }
			MapType GetSensorView() { return sensor_view_; }

			void GetRobotLocalMap(MapType &local_map) {
				local_map = MapType::Zero(pLocalMapSize, pLocalMapSize);
				if(MapUtils::IsPointOutsideBoundary(global_current_position_, pRobotMapSize, pWorldMapSize)) {
					return;
				}
				MapUtils::GetSubMap(global_current_position_, pLocalMapSize, pRobotMapSize, robot_map_, local_map);
			}

	};

} /* namespace CoverageControl */
#endif /* COVERAGECONTROL_ROBOTMODEL_H_ */

