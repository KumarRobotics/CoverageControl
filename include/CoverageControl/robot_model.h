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

namespace coveragecontrol {

	class RobotModel {

		private:

			Point2 global_start_position_, global_current_position_;
			Point2 local_start_position_, local_current_position_;
			std::vector <Point2> robot_positions_; // Stores the local positions of the robot in order


			MapType robot_map_; // Stores what the robot has seen. Has the same reference as world map.
			WorldIDF world_idf_;

			// Gets the closest point on the grid
			Point2 GetPointOnGrid(Point2 pt) {
				pt = pt/pResolution;
				pt = Point2(std::round(pt.x()), std::round(pt.y())) * pResolution;
				return pt;
			}

			// Gets the sensor data from world IDF at the global_current_position_ and updates robot_map_
			void UpdateRobotMap() {
				auto sensor_data = world_idf_.GetSensorData(GetPointOnGrid(global_current_position_), pSensorSize);
			}

		public:

			// Constructor: requires global_start_position_ and world_idf_
			RobotModel (Point2 const &global_start_position, WorldIDF const &world_idf):
				global_start_position_{global_start_position},
				world_idf_{world_idf} {

				local_current_position_ = Point2{0,0};
				robot_positions_.reserve(pEpisodeSteps); // More steps can be executed. But reserving for efficiency
				local_current_position_ = local_start_position_;
				robot_positions_.push_back(local_current_position_);
				robot_map_ = MapType::Constant(pRobotMapSize, pRobotMapSize, pUnknownImportance);
				UpdateRobotMap();
			}

			Point2 GetGlobalStartPosition() { return global_start_position_; }
			Point2 GetGlobalCurrentPosition() { return global_current_position_; }
			std::vector <Point2> GetAllPositions() { return robot_positions_; }

			MapType GetRobotMap() { return robot_map_; }

	};

} /* namespace coveragecontrol */
#endif /* COVERAGECONTROL_ROBOTMODEL_H_ */

