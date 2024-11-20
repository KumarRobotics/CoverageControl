/*
 * This file is part of the CoverageControl library
 *
 * Author: Saurav Agarwal
 * Contact: sauravag@seas.upenn.edu, agr.saurav1@gmail.com
 * Repository: https://github.com/KumarRobotics/CoverageControl
 *
 * Copyright (c) 2024, Saurav Agarwal
 *
 * The CoverageControl library is free software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * The CoverageControl library is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * CoverageControl library. If not, see <https://www.gnu.org/licenses/>.
 */

/*!
 * \file robot_model.h
 * \brief Class for handling the robot model
 *
 * The robot model is used to simulate the robot's movement and sensor data.
 * Makes heavy use of \ref parameters.h
 *
 */

#ifndef CPPSRC_CORE_INCLUDE_COVERAGECONTROL_ROBOT_MODEL_H_
#define CPPSRC_CORE_INCLUDE_COVERAGECONTROL_ROBOT_MODEL_H_

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

#include "CoverageControl/constants.h"
#include "CoverageControl/map_utils.h"
#include "CoverageControl/parameters.h"
#include "CoverageControl/typedefs.h"
#include "CoverageControl/voronoi.h"
#include "CoverageControl/world_idf.h"

namespace CoverageControl {

/*!
 * \addtogroup cpp_api
 * @{
 * \class RobotModel
 * @}
 * \brief Class for handling the robot model
 *
 * The robot model is used to simulate the robot's movement and sensor data.
 *
 */
class RobotModel {
 private:
  Parameters const params_;

  Point2 global_start_position_, global_current_position_;
  Point2 local_start_position_, local_current_position_;
  double normalization_factor_ = 0;

  MapType robot_map_;     //!< Stores what the robot has seen. Has the same
                          //!< reference as world map.
  MapType sensor_view_;   //!< Stores the current sensor view of the robot
  MapType local_map_;     //!< Stores the local map of the robot
  MapType obstacle_map_;  //!< Stores the obstacle map
  MapType system_map_;    //!< Stores the obstacle map
  MapType
      local_exploration_map_;  //!< Binary map: true for unexplored locations
  MapType exploration_map_;    //!< Binary map: true for unexplored locations
  double time_step_dist_ = 0;
  double sensor_area_ = 0;

  std::shared_ptr<const WorldIDF>
      world_idf_;  //!< Robots cannot change the world
  // const WorldIDF *world_idf_;  //!< Robots cannot change the world

  // Gets the sensor data from world IDF at the global_current_position_ and
  // updates robot_map_
  void UpdateSensorView() {
    sensor_view_ = MapType::Zero(params_.pSensorSize, params_.pSensorSize);
    if (MapUtils::IsPointOutsideBoundary(
            params_.pResolution, global_current_position_, params_.pSensorSize,
            params_.pWorldMapSize)) {
      return;
    }
    world_idf_->GetSubWorldMap(global_current_position_, params_.pSensorSize,
                               sensor_view_);
  }

  void UpdateRobotMap() {
    MapUtils::MapBounds index, offset;
    MapUtils::ComputeOffsets(params_.pResolution, global_current_position_,
                             params_.pSensorSize, params_.pWorldMapSize, index,
                             offset);
    robot_map_.block(index.left + offset.left, index.bottom + offset.bottom,
                     offset.width, offset.height) =
        sensor_view_.block(offset.left, offset.bottom, offset.width,
                           offset.height);
  }

  void UpdateExplorationMap() {
    if (MapUtils::IsPointOutsideBoundary(
            params_.pResolution, global_current_position_, params_.pSensorSize,
            params_.pWorldMapSize)) {
      return;
    }
    MapUtils::MapBounds index, offset;
    MapUtils::ComputeOffsets(params_.pResolution, global_current_position_,
                             params_.pSensorSize, params_.pWorldMapSize, index,
                             offset);
    exploration_map_.block(
        index.left + offset.left, index.bottom + offset.bottom, offset.width,
        offset.height) = MapType::Zero(offset.width, offset.height);
  }

  void Initialize() {
    normalization_factor_ = world_idf_->GetNormalizationFactor();
    global_current_position_ = global_start_position_;

    sensor_view_ = MapType::Zero(params_.pSensorSize, params_.pSensorSize);
    local_map_ = MapType::Zero(params_.pLocalMapSize, params_.pLocalMapSize);
    obstacle_map_ = MapType::Zero(params_.pLocalMapSize, params_.pLocalMapSize);

    local_start_position_ = Point2{0, 0};
    local_current_position_ = local_start_position_;

    ClearRobotMap();

    if (params_.pUpdateExplorationMap == true) {
      exploration_map_ =
          MapType::Constant(params_.pRobotMapSize, params_.pRobotMapSize, 1);
      local_exploration_map_ =
          MapType::Constant(params_.pRobotMapSize, params_.pRobotMapSize, 1);
      UpdateExplorationMap();
    } else {
      exploration_map_ =
          MapType::Constant(params_.pRobotMapSize, params_.pRobotMapSize, 0);
      local_exploration_map_ =
          MapType::Constant(params_.pRobotMapSize, params_.pRobotMapSize, 0);
    }

    time_step_dist_ =
        params_.pMaxRobotSpeed * params_.pTimeStep * params_.pResolution;
    sensor_area_ = params_.pSensorSize * params_.pSensorSize;
  }

 public:
  /*!
   * \brief Constructor for the robot model
   *
   * \param params Parameters for the robot model
   * \param global_start_position The global start position of the robot
   * \param world_idf The world IDF object
   */

  RobotModel(Parameters const &params, Point2 const &global_start_position,
             std::shared_ptr<const WorldIDF> const &world_idf)
      : params_{params},
        global_start_position_{global_start_position},
        world_idf_{world_idf} {
    Initialize();
  }

  RobotModel(Parameters const &params, Point2 const &global_start_position,
             WorldIDF const &world_idf)
      : params_{params},
        global_start_position_{global_start_position},
        world_idf_{std::make_shared<const WorldIDF>(world_idf)} {
    Initialize();
  }

  void ClearRobotMap() {
    if (params_.pRobotMapUseUnknownImportance == true) {
      robot_map_ =
          MapType::Constant(params_.pRobotMapSize, params_.pRobotMapSize,
                            params_.pUnknownImportance * params_.pNorm);
    } else {
      robot_map_ = MapType::Zero(params_.pRobotMapSize, params_.pRobotMapSize);
    }

    if (params_.pUpdateSensorView == true) {
      UpdateSensorView();
    }
    if (params_.pUpdateRobotMap == true) {
      UpdateRobotMap();
    }
  }

  //! \note Time step robot with the given control direction and speed.
  //! \note Direction cannot be zero-vector is speed is not zero.
  //! \note Speed needs to be positive
  int StepControl(Point2 const &direction, double const &speed) {
    auto dir = direction;
    auto sp = speed;
    Point2 new_pos = local_current_position_;
    if (sp > params_.pMaxRobotSpeed) {
      sp = params_.pMaxRobotSpeed;
    }
    bool invalid_dir = dir.norm() < kEps and sp >= kEps;
    if (sp < 0 or invalid_dir) {
      std::cout << sp << " " << dir.norm() << std::endl;
      std::cerr << "Speed needs to be non-negative\n";
      std::cerr << "Zero-vector direction cannot be given in control\n";
      std::throw_with_nested(
          std::runtime_error("Speed needs to be non-negative"));
      return 1;
    }
    /* if (dir.normalize() and sp > kEps) { */
    /*   std::cerr << "Zero-vector direction given in control\n"; */
    /*   return 1; */
    /* } */
    dir.normalize();
    if (sp > kEps) {
      new_pos = local_current_position_ + dir * sp * params_.pTimeStep;
    }
    SetRobotPosition(new_pos);
    return 0;
  }

  //! Set robot position relative to the current position
  void SetRobotPosition(Point2 const &pos) {
    Point2 new_global_pos = pos + global_start_position_;
    if (new_global_pos.x() <= 0) {
      new_global_pos[0] = 0 + kLargeEps;
    }
    if (new_global_pos.y() <= 0) {
      new_global_pos[1] = 0 + kLargeEps;
    }
    double max_xy = params_.pWorldMapSize * params_.pResolution;
    if (new_global_pos.x() >= max_xy) {
      new_global_pos[0] = max_xy - kLargeEps;
    }
    if (new_global_pos.y() >= max_xy) {
      new_global_pos[1] = max_xy - kLargeEps;
    }

    local_current_position_ = new_global_pos - global_start_position_;
    global_current_position_ = new_global_pos;
    if (params_.pUpdateSensorView == true) {
      UpdateSensorView();
    }
    if (params_.pUpdateRobotMap == true) {
      UpdateRobotMap();
    }
    if (params_.pUpdateExplorationMap == true) {
      UpdateExplorationMap();
    }
  }

  void SetGlobalRobotPosition(Point2 const &pos) {
    Point2 new_global_pos = pos;
    if (new_global_pos.x() <= 0) {
      new_global_pos[0] = 0 + kLargeEps;
    }
    if (new_global_pos.y() <= 0) {
      new_global_pos[1] = 0 + kLargeEps;
    }
    double max_xy = params_.pWorldMapSize * params_.pResolution;
    if (new_global_pos.x() >= max_xy) {
      new_global_pos[0] = max_xy - kLargeEps;
    }
    if (new_global_pos.y() >= max_xy) {
      new_global_pos[1] = max_xy - kLargeEps;
    }

    local_current_position_ = new_global_pos - global_start_position_;
    global_current_position_ = new_global_pos;
    if (params_.pUpdateSensorView == true) {
      UpdateSensorView();
    }
    if (params_.pUpdateRobotMap == true) {
      UpdateRobotMap();
    }
    if (params_.pUpdateExplorationMap == true) {
      UpdateExplorationMap();
    }
  }

  Point2 GetGlobalStartPosition() const { return global_start_position_; }
  Point2 GetGlobalCurrentPosition() const { return global_current_position_; }

  const MapType &GetRobotMap() const { return robot_map_; }
  const MapType &GetSensorView() const { return sensor_view_; }

  const MapType &GetRobotSystemMap() {
    GetRobotLocalMap();
    GetExplorationMap();
    system_map_ = local_map_ - local_exploration_map_;
    return system_map_;
  }

  // const MapType &GetRobotLocalMap() {
  /* local_map_ = MapType::Constant(params_.pLocalMapSize,
   * params_.pLocalMapSize, -1.0); */
  void ComputeLocalMap() {
    local_map_ =
        MapType::Constant(params_.pLocalMapSize, params_.pLocalMapSize, 0.);
    if (not MapUtils::IsPointOutsideBoundary(
            params_.pResolution, global_current_position_,
            params_.pLocalMapSize, params_.pWorldMapSize)) {
      MapUtils::GetSubMap(params_.pResolution, global_current_position_,
                          params_.pRobotMapSize, robot_map_,
                          params_.pLocalMapSize, local_map_);
    }
  }
  MapType &GetRobotMapMutable() { return robot_map_; }

  const MapType &GetRobotLocalMap() {
    ComputeLocalMap();
    return local_map_;
  }

  const MapType &GetExplorationMap() {
    /* local_exploration_map_ = MapType::Constant(params_.pLocalMapSize,
     * params_.pLocalMapSize, 0); */
    local_exploration_map_ =
        MapType::Constant(params_.pLocalMapSize, params_.pLocalMapSize, -1.0);
    if (not MapUtils::IsPointOutsideBoundary(
            params_.pResolution, global_current_position_,
            params_.pLocalMapSize, params_.pWorldMapSize)) {
      MapUtils::GetSubMap(params_.pResolution, global_current_position_,
                          params_.pRobotMapSize, exploration_map_,
                          params_.pLocalMapSize, local_exploration_map_);
    }
    return local_exploration_map_;
  }

  const MapType &GetObstacleMap() {
    obstacle_map_ = MapType::Ones(params_.pLocalMapSize, params_.pLocalMapSize);
    MapUtils::MapBounds index, offset;
    ComputeOffsets(params_.pResolution, global_current_position_,
                   params_.pLocalMapSize, params_.pRobotMapSize, index, offset);
    obstacle_map_.block(offset.left, offset.bottom, offset.width,
                        offset.height) =
        MapType::Zero(offset.width, offset.height);
    return obstacle_map_;
  }

  auto GetExplorationFeatures() {
    queue_t frontiers;
    Point2 const &pos = global_current_position_;

    double factor = 4;
    double pos_x_lim = std::min(pos[0] + factor * time_step_dist_,
                                static_cast<double>(params_.pWorldMapSize));
    double pos_y_lim = std::min(pos[1] + factor * time_step_dist_,
                                static_cast<double>(params_.pWorldMapSize));
    for (double pos_x = std::max(pos[0] - factor * time_step_dist_, kLargeEps);
         pos_x < pos_x_lim; pos_x += 1 * params_.pResolution) {
      for (double pos_y =
               std::max(pos[1] - factor * time_step_dist_, kLargeEps);
           pos_y < pos_y_lim; pos_y += 1 * params_.pResolution) {
        Point2 qpos{pos_x, pos_y};
        double dist = (qpos - pos).norm();
        dist = std::max(dist, time_step_dist_);
        MapUtils::MapBounds index, offset;
        MapUtils::ComputeOffsets(params_.pResolution, qpos, params_.pSensorSize,
                                 params_.pWorldMapSize, index, offset);
        double unexplored =
            exploration_map_
                .block(index.left + offset.left, index.bottom + offset.bottom,
                       offset.width, offset.height)
                .sum();
        double benefit = unexplored;
        if (unexplored > 0.1 * sensor_area_) {
          double idf_value =
              robot_map_
                  .block(index.left + offset.left, index.bottom + offset.bottom,
                         offset.width, offset.height)
                  .count();
          benefit += 2 * idf_value;
        }
        double bcr = benefit / dist;

        if (frontiers.size() < size_t(params_.pNumFrontiers)) {
          frontiers.push(Frontier(qpos, bcr));
        } else {
          auto worst_frontier = frontiers.top();
          if (worst_frontier.value < bcr) {
            frontiers.pop();
            frontiers.push(Frontier(qpos, bcr));
          }
        }
      }
    }
    std::vector<double> frontier_features(params_.pNumFrontiers * 2);
    int count = 0;
    while (!frontiers.empty()) {
      auto point = frontiers.top();
      frontier_features[count++] = point.pt[0] - pos[0];
      frontier_features[count++] = point.pt[1] - pos[1];
      frontiers.pop();
    }
    return frontier_features;
  }

  /* The centroid is computed with orgin of the map, i.e., the lower left corner
   * of the map. */
  /* Only the local map is considered for the computation of the centroid. */
  auto GetVoronoiFeatures() {
    auto const &pos = global_current_position_;
    MapUtils::MapBounds index, offset;
    MapUtils::ComputeOffsets(params_.pResolution, pos, params_.pLocalMapSize,
                             params_.pWorldMapSize, index, offset);
    auto trimmed_local_map =
        robot_map_.block(index.left + offset.left, index.bottom + offset.bottom,
                         offset.width, offset.height);

    Point2 map_size(offset.width, offset.height);
    Point2 map_translation(
        (index.left + offset.left) * params_.pResolution,
        (index.bottom + offset.bottom) * params_.pResolution);

    PointVector robot_positions(1);
    robot_positions[0] = pos - map_translation;
    Voronoi voronoi(robot_positions, trimmed_local_map, map_size,
                    params_.pResolution, true, 0);
    auto vcell = voronoi.GetVoronoiCell();
    /* vcell.centroid += Point2(params_.pLocalMapSize / 2.,
     * params_.pLocalMapSize / 2.); */
    return vcell.GetFeatureVector();
  }
};

} /* namespace CoverageControl */
#endif  // CPPSRC_CORE_INCLUDE_COVERAGECONTROL_ROBOT_MODEL_H_
