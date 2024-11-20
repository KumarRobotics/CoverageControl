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
 * \file coverage_system.h
 * \brief The file contains the CoverageSystem class, which is the main class
 * for the coverage control library.
 */

#ifndef CPPSRC_CORE_INCLUDE_COVERAGECONTROL_COVERAGE_SYSTEM_H_
#define CPPSRC_CORE_INCLUDE_COVERAGECONTROL_COVERAGE_SYSTEM_H_

#define EIGEN_NO_CUDA  // Don't use eigen's cuda facility
#include <omp.h>

#include <Eigen/Dense>  // Eigen is used for maps
#include <algorithm>
#include <iostream>
#include <list>
#include <mutex>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "CoverageControl/bivariate_normal_distribution.h"
#include "CoverageControl/constants.h"
#include "CoverageControl/map_utils.h"
#include "CoverageControl/parameters.h"
#include "CoverageControl/plotter.h"
#include "CoverageControl/robot_model.h"
#include "CoverageControl/typedefs.h"
#include "CoverageControl/voronoi.h"
#include "CoverageControl/world_idf.h"

namespace CoverageControl {

/*!
 * \addtogroup cpp_api
 * @{
 * \class CoverageSystem
 * @}
 * \brief The CoverageSystem class is the main class for the coverage control
 * library.
 */
class CoverageSystem {
  Parameters const params_;          //!< Parameters for the coverage system
  std::shared_ptr <WorldIDF> world_idf_ptr_; //!< World IDF object
  size_t num_robots_ = 0;            //!< Number of robots
  std::vector<RobotModel> robots_;   //!< Vector of robots of type RobotModel
  double normalization_factor_ = 0;  //!< Normalization factor for the world IDF
  Voronoi voronoi_;                  //!< Voronoi object
  std::vector<VoronoiCell> voronoi_cells_;  //!< Voronoi cells for each robot
  mutable std::random_device
      rd_;                    //!< Random device for random number generation
  mutable std::mt19937 gen_;  //!< Mersenne Twister random number generator
  mutable std::mutex mutex_;
  std::uniform_real_distribution<>
      distrib_pts_;  //!< Uniform distribution for generating random points
  PointVector robot_global_positions_;  //!< Global positions of the robots
  MapType
      system_map_;  //!< System map contains explored and unexplored locations
  MapType
      exploration_map_;  //!< Exploration map contains the unexplored locations
  MapType
      explored_idf_map_;  //!< Explored IDF map contains the explored locations
  std::vector<std::list<Point2>>
      robot_positions_history_;   //!< History of robot positions
  double exploration_ratio_ = 0;  //!< Ratio of explored locations
  double weighted_exploration_ratio_ =
      0;                         //!< Weighted ratio of explored locations
  double total_idf_weight_ = 0;  //!< Total weight of the world IDF
  std::vector<PlotterData> plotter_data_;  //!< Stores data for plotting
  std::vector<std::vector<Point2>>
      relative_positions_neighbors_;  //!< Relative positions of neighboring
                                      //!< robots for each robot
  std::vector<std::vector<int>>
      neighbor_ids_;  //!< IDs of neighboring robots for each robot

  //! Initialize the member variables
  void InitSetup();

  //! Update the exploration map, explored IDF map, and system map
  void UpdateSystemMap() {
    // This is not necessarily thread safe. Do NOT parallelize this for loop
    for (size_t i = 0; i < num_robots_; ++i) {
      MapUtils::MapBounds index, offset;
      MapUtils::ComputeOffsets(params_.pResolution, robot_global_positions_[i],
                               params_.pSensorSize, params_.pWorldMapSize,
                               index, offset);
      explored_idf_map_.block(index.left + offset.left,
                              index.bottom + offset.bottom, offset.width,
                              offset.height) =
          GetRobotSensorView(i).block(offset.left, offset.bottom, offset.width,
                                      offset.height);
      exploration_map_.block(
          index.left + offset.left, index.bottom + offset.bottom, offset.width,
          offset.height) = MapType::Zero(offset.width, offset.height);
    }
    system_map_ = explored_idf_map_ - exploration_map_;
    /* exploration_ratio_ = 1.0 -
     * (double)(exploration_map_.sum())/(params_.pWorldMapSize *
     * params_.pWorldMapSize); */
    /* weighted_exploration_ratio_ =
     * (double)(explored_idf_map_.sum())/(total_idf_weight_); */
    /* std::cout << "Exploration: " << exploration_ratio_ << " Weighted: " <<
     * weighted_exploration_ratio_ << std::endl; */
    /* std::cout << "Diff: " << (exploration_map_.count() -
     * exploration_map_.sum()) << std::endl; */
  }

  //! Execute updates after a step for robot_id (avoid using this function, use
  //! PostStepCommands() instead).
  void PostStepCommands(size_t robot_id);

  //! Execute updates after a step for all robots
  //! \note This function should be called after every step to update the state
  //! of the system
  void PostStepCommands();

  //! Compute the adjacency matrix for communication
  void ComputeAdjacencyMatrix();

  //! Update the positions of all robots from the RobotModel objects
  void UpdateRobotPositions() {
    for (size_t iRobot = 0; iRobot < num_robots_; ++iRobot) {
      robot_global_positions_[iRobot] =
          robots_[iRobot].GetGlobalCurrentPosition();
    }
  }

  //! Update the neighbors of all robots
  void UpdateNeighbors();

 public:
  //! \name Constructors
  //! @{

  /*! Delete the default constructor */
  CoverageSystem() = delete;

  /*! \brief Create random Gaussian distributions for the world IDF and random
   * start positions for the robots
   *
   * \param params Parameters for the coverage system
   */
  explicit CoverageSystem(Parameters const &params);

  /*! \brief Create random Gaussian distributions for the world IDF and random
   * start positions for the robots
   *
   * \param params Parameters for the coverage system
   * \param num_gaussians Number of gaussian distributions for the world IDF
   * \param num_robots Number of robots
   */
  CoverageSystem(Parameters const &params, int const num_gaussians,
                 int const num_robots);

  CoverageSystem(Parameters const &params, int const num_gaussians,
                 int const num_polygons, int const num_robots);

  /*! \brief Create with given world IDF and robot positions from file
   *
   *
   * \param params Parameters for the coverage system
   * \param world_idf World IDF
   * \param pos_file_name File name for initial positions
   */
  CoverageSystem(Parameters const &params, WorldIDF const &world_idf,
                 std::string const &pos_file_name);

  /*! \brief Constructor for given world IDF and robot positions as a vector
   *
   * \param params Parameters for the coverage system
   * \param world_idf World IDF
   * \param robot_positions Initial positions of the robots
   */
  CoverageSystem(Parameters const &params, WorldIDF const &world_idf,
                 std::vector<Point2> const &robot_positions);

  /*! \brief Constructor for given normal distributions and robot positions
   *
   * \param params Parameters for the coverage system
   * \param dists Bivariate normal distributions for the world IDF
   * \param robot_positions Initial positions of the robots
   */
  CoverageSystem(Parameters const &params,
                 std::vector<BivariateNormalDistribution> const &dists,
                 std::vector<Point2> const &robot_positions);

  //! @}

  //! \name Setters
  //! @{

  //! Set the positions of all robots with respect to their current positions
  //! \note Same as SetRobotPositions
  void SetLocalRobotPositions(std::vector<Point2> const &relative_pos) {
    SetRobotPositions(relative_pos);
  }

  //! Set the position of robot_id with respect to its current position
  void SetLocalRobotPosition(size_t const robot_id,
                             Point2 const &relative_pos) {
    robots_[robot_id].SetRobotPosition(relative_pos);
    PostStepCommands(robot_id);
  }

  //! Set the global position of robot_id
  void SetGlobalRobotPosition(size_t const robot_id, Point2 const &global_pos) {
    robots_[robot_id].SetGlobalRobotPosition(global_pos);
    PostStepCommands(robot_id);
  }
  //
  //! Set the global positions of all robots
  void SetGlobalRobotPositions(PointVector const &global_positions) {
    if (global_positions.size() != num_robots_) {
      throw std::length_error{
          "The size of the positions don't match with the number of robots"};
    }
    for (size_t i = 0; i < num_robots_; ++i) {
      robots_[i].SetGlobalRobotPosition(global_positions[i]);
    }
    PostStepCommands();
  }

  //! Set the positions of all robots with respect to their current positions
  //! \note Same as SetLocalRobotPositions
  void SetRobotPositions(std::vector<Point2> const &positions) {
    if (positions.size() != num_robots_) {
      throw std::length_error{
          "The size of the positions don't match with the number of robots"};
    }
    for (size_t i = 0; i < num_robots_; ++i) {
      robots_[i].SetRobotPosition(positions[i]);
    }
    PostStepCommands();
  }

  //! Set the world IDF and recompute the world map
  void SetWorldIDF(WorldIDF const &world_idf) {
    world_idf_ptr_.reset(new WorldIDF{world_idf});
    normalization_factor_ = world_idf_ptr_->GetNormalizationFactor();
  }
  //! @}

  //! \name Robot related functions
  //! @{

  /*!
   * \brief Execute given actions for all robots
   *
   * \warning If the function returns 1 (control is incorrect), the system state
   * is not updated
   *
   * \param actions Vector of actions for all robots
   * \return 0 if successful, 1 if control is incorrect
   */
  [[nodiscard]] bool StepActions(PointVector const &actions) {
    for (size_t iRobot = 0; iRobot < num_robots_; ++iRobot) {
      Point2 action = actions[iRobot];
      double speed = action.norm();
      Point2 direction = action.normalized();
      if (robots_[iRobot].StepControl(direction, speed)) {
        std::cerr << "Control incorrect\n";
        return 1;
      }
    }
    PostStepCommands();
    return 0;
  }

  /*!
   * \brief Execute given action for robot_id
   *
   * \warning If the function returns 1 (control is incorrect), the system state
   * is not updated
   *
   * \param robot_id ID of the robot
   * \param action Action for the robot
   * \return 0 if successful, 1 if control is incorrect
   */
  [[nodiscard]] bool StepAction(size_t const robot_id, Point2 const action) {
    double speed = action.norm();
    Point2 direction = action.normalized();
    if (robots_[robot_id].StepControl(direction, speed)) {
      std::cerr << "Control incorrect\n";
      return 1;
    }
    PostStepCommands();
    return 0;
  }

  /*!
   * \brief Execute velocity control for robot_id
   *
   * \warning If the function returns 1 (control is incorrect), the system state
   * is not updated
   *
   * \param robot_id ID of the robot
   * \param direction Velocity direction
   * \param speed Velocity magnitude
   * \return 0 if successful, 1 if control is incorrect
   */
  [[nodiscard]] bool StepControl(size_t robot_id, Point2 const &direction,
                                 double const speed) {
    if (robots_[robot_id].StepControl(direction, speed)) {
      std::cerr << "Control incorrect\n";
      return 1;
    }
    PostStepCommands();
    return 0;
  }

  //! Add noise to the given point and ensure within bounds
  Point2 AddNoise(Point2 const pt) const;

  //! Check if the robot is oscillating about its current position
  //! \warning This function is dependent on the size of the robot positions
  //! history
  bool CheckOscillation(size_t const robot_id) const {
    if (params_.pCheckOscillations == false) {
      return false;
    }
    if (robot_positions_history_[robot_id].size() < 3) {
      return false;
    }
    auto const &history = robot_positions_history_[robot_id];
    Point2 const last_pos = history.back();
    auto it_end = std::next(history.crbegin(), std::min(6, static_cast<int>(history.size()) - 1));
    bool flag = false;
    int count = 0;
    std::for_each(history.crbegin(), it_end,
                  [last_pos, &count](Point2 const &pt) {
                    if ((pt - last_pos).norm() < kLargeEps) {
                      ++count;
                    }
                  });
    if (count > 2) {
      flag = true;
    }
    return flag;
  }

  void CheckRobotID(size_t const id) const {
    if (id >= num_robots_) {
      throw std::out_of_range{"Robot index more than the number of robots"};
    }
  }

  void ComputeVoronoiCells() {
    UpdateRobotPositions();
    voronoi_ = Voronoi(robot_global_positions_, GetWorldMap(),
                       Point2(params_.pWorldMapSize, params_.pWorldMapSize),
                       params_.pResolution);
    voronoi_cells_ = voronoi_.GetVoronoiCells();
  }

  /*!
   * Step a robot towards a given goal
   *
   * \param robot_id ID of the robot
   * \param goal Goal for the robot
   * \param speed_factor Speed factor for the robots
   * \return True if any robot is still moving
   */
  bool StepRobotToGoal(int const robot_id, Point2 const &goal,
                       double const speed_factor = 1);

  /*!
   * Step all robots towards given goals
   *
   * \param goals Vector of goals for the robots
   * \param actions Vector of actions for the robots
   * \return True if any robot is still moving
   */
  bool StepRobotsToGoals(PointVector const &goals, PointVector &actions);

  void ClearRobotMaps() {
    for (size_t i = 0; i < num_robots_; ++i) {
      robots_[i].ClearRobotMap();
    }
  }

  void ClearExploredIDF() {
    explored_idf_map_ =
        MapType::Constant(params_.pWorldMapSize, params_.pWorldMapSize, 0);
  }

  //! @}

  //! \name I/O
  //! @{
  int WriteRobotPositions(std::string const &file_name) const;
  int WriteRobotPositions(std::string const &file_name,
                          PointVector const &positions) const;
  int WriteEnvironment(std::string const &pos_filename,
                       std::string const &env_filename) const;
  //! @}

  //! \name Plot related functions
  //! @{
  void PlotFrontiers(std::string const &, int const &,
                     PointVector const &) const;
  void PlotSystemMap(std::string const &dir_name, int const &step) const {
    std::vector<int> robot_status(num_robots_, 0);
    PlotSystemMap(dir_name, step, robot_status);
  }
  void PlotSystemMap(std::string const &) const;
  void PlotSystemMap(std::string const &, int const &,
                     std::vector<int> const &) const;
  void PlotMapVoronoi(std::string const &, int const &);
  void PlotMapVoronoi(std::string const &, int const &, Voronoi const &,
                      PointVector const &) const;
  void PlotWorldMap(std::string const &, std::string const &) const;
  void PlotWorldMapRobots(std::string const &, std::string const &) const;
  void PlotInitMap(std::string const &filename) const {
    PlotInitMap("./", filename);
  }
  void PlotInitMap(std::string const &, std::string const &) const;
  void PlotRobotLocalMap(std::string const &, int const &, int const &);
  void PlotRobotSystemMap(std::string const &, int const &, int const &);
  void PlotRobotExplorationMap(std::string const &, int const &, int const &);
  void PlotRobotSensorView(std::string const &, int const &, int const &);
  void PlotRobotObstacleMap(std::string const &, int const &, int const &);
  void PlotRobotCommunicationMaps(std::string const &, int const &, int const &,
                                  size_t const &);

  void RenderRecordedMap(std::string const &, std::string const &) const;
  void RecordPlotData(std::vector<int> const &, std::string const &);
  void RecordPlotData(std::vector<int> const &robot_status) {
    RecordPlotData(robot_status, "system");
  }
  void RecordPlotData(std::string const &map_name) {
    std::vector<int> robot_status(num_robots_, 0);
    RecordPlotData(robot_status, map_name);
  }
  void RecordPlotData() {
    std::vector<int> robot_status(num_robots_, 0);
    RecordPlotData(robot_status, "system");
  }
  //! @}

  //! \name Getters
  //
  //! @{
  std::shared_ptr<const WorldIDF> GetWorldIDFPtr() const {
    return world_idf_ptr_;
  }
  const WorldIDF &GetWorldIDFObject() const { return *world_idf_ptr_; }
  const MapType &GetSystemMap() const { return system_map_; }
  const MapType &GetSystemExplorationMap() const { return exploration_map_; }
  const MapType &GetSystemExploredIDFMap() const { return explored_idf_map_; }
  MapType &GetSystemExploredIDFMapMutable() { return explored_idf_map_; }
  //! Get the world map
  const MapType &GetWorldMap() const { return world_idf_ptr_->GetWorldMap(); }
  //! Get the world map (mutable)
  MapType &GetWorldMapMutable() { return world_idf_ptr_->GetWorldMapMutable(); }

  MapType &GetRobotMapMutable(size_t const id) {
    CheckRobotID(id);
    return robots_[id].GetRobotMapMutable();
  }

  inline auto GetNumRobots() const { return num_robots_; }
  inline auto GetNumFeatures() const { return num_robots_; }

  inline double GetExplorationRatio() const {
    double exploration_ratio =
        1.0 - static_cast<double>(exploration_map_.sum()) /
                  (params_.pWorldMapSize * params_.pWorldMapSize);
    return exploration_ratio;
  }

  //! Get the weighted (by IDF) exploration ratio
  inline double GetWeightedExplorationRatio() const {
    double weighted_exploration_ratio =
        static_cast<double>(explored_idf_map_.sum()) / (total_idf_weight_);
    return weighted_exploration_ratio;
  }

  PointVector GetRelativePositonsNeighbors(size_t const robot_id);
  std::vector<int> GetNeighborIDs(size_t const robot_id) const {
    return neighbor_ids_[robot_id];
  }

  /*!
   * \brief Get the global positions of all robots
   *
   * Can add noise to the positions based on the parameters
   * \param force_no_noise If true, returns the positions without noise
   * \return Vector of global positions of all robots
   */
  PointVector GetRobotPositions(bool force_no_noise = false) {
    UpdateRobotPositions();
    if (params_.pAddNoisePositions and not force_no_noise) {
      PointVector noisy_robot_global_positions;
      for (Point2 pt : robot_global_positions_) {
        noisy_robot_global_positions.push_back(AddNoise(pt));
      }
      return noisy_robot_global_positions;
    }
    return robot_global_positions_;
  }

  Point2 GetRobotPosition(int const robot_id,
                          bool force_no_noise = false) const {
    Point2 robot_pos;
    robot_pos[0] = robots_[robot_id].GetGlobalCurrentPosition()[0];
    robot_pos[1] = robots_[robot_id].GetGlobalCurrentPosition()[1];
    if (params_.pAddNoisePositions and not force_no_noise) {
      return AddNoise(robot_pos);
    }
    return robot_pos;
  }

  const MapType &GetRobotLocalMap(size_t const id) {
    CheckRobotID(id);
    return robots_[id].GetRobotLocalMap();
  }

  const MapType &GetRobotMap(size_t const id) const {
    CheckRobotID(id);
    return robots_[id].GetRobotMap();
  }

  const MapType &GetRobotExplorationMap(size_t const id) {
    CheckRobotID(id);
    return robots_[id].GetExplorationMap();
  }

  const MapType &GetRobotSystemMap(size_t const id) {
    CheckRobotID(id);
    return robots_[id].GetRobotSystemMap();
  }
  const MapType &GetRobotObstacleMap(size_t const id) {
    CheckRobotID(id);
    return robots_[id].GetObstacleMap();
  }

  const MapType &GetRobotSensorView(size_t const id) const {
    CheckRobotID(id);
    return robots_[id].GetSensorView();
  }

  auto GetRobotsInCommunication(size_t const id) const {
    CheckRobotID(id);
    PointVector robot_neighbors_pos;
    for (size_t i = 0; i < num_robots_; ++i) {
      if (id == i) {
        continue;
      }
      Point2 relative_pos =
          robot_global_positions_[i] - robot_global_positions_[id];
      if (relative_pos.norm() < params_.pCommunicationRange) {
        robot_neighbors_pos.push_back(relative_pos);
      }
    }
    std::sort(
        robot_neighbors_pos.begin(), robot_neighbors_pos.end(),
        [](Point2 const &a, Point2 const &b) { return a.norm() < b.norm(); });
    return robot_neighbors_pos;
  }

  std::pair<MapType, MapType> GetRobotCommunicationMaps(size_t const, size_t);

  std::vector<MapType> GetCommunicationMaps(size_t map_size) {
    std::vector<MapType> communication_maps(2 * num_robots_);
    /* #pragma omp parallel for num_threads(num_robots_) */
    for (size_t i = 0; i < num_robots_; ++i) {
      auto comm_map = GetRobotCommunicationMaps(i, map_size);
      communication_maps[2 * i] = comm_map.first;
      communication_maps[2 * i + 1] = comm_map.second;
    }
    return communication_maps;
  }

  auto GetObjectiveValue() {
    ComputeVoronoiCells();
    return voronoi_.GetSumIDFSiteDistSqr();
  }

  auto GetRobotExplorationFeatures() {
    std::vector<std::vector<double>> features(num_robots_);
#pragma omp parallel for num_threads(num_robots_)
    for (size_t iRobot = 0; iRobot < num_robots_; ++iRobot) {
      features[iRobot] = robots_[iRobot].GetExplorationFeatures();
    }
    return features;
  }

  auto GetRobotVoronoiFeatures() {
    std::vector<std::vector<double>> features(num_robots_);
#pragma omp parallel for num_threads(num_robots_)
    for (size_t iRobot = 0; iRobot < num_robots_; ++iRobot) {
      features[iRobot] = robots_[iRobot].GetVoronoiFeatures();
    }
    return features;
  }

  //! \note The centroid is computed with orgin of the map, i.e., the lower left
  //! corner of the map. \note Uses neighboring robots' positions to compute the
  //! centroid.
  std::vector<double> GetLocalVoronoiFeatures(int const robot_id);

  std::vector<std::vector<double>> GetLocalVoronoiFeatures() {
    std::vector<std::vector<double>> features(num_robots_);
#pragma omp parallel for num_threads(num_robots_)
    for (size_t iRobot = 0; iRobot < num_robots_; ++iRobot) {
      features[iRobot] = GetLocalVoronoiFeatures(iRobot);
    }
    return features;
  }

  auto GetVoronoiCells() { return voronoi_cells_; }
  Voronoi &GetVoronoi() { return voronoi_; }

  auto GetVoronoiCell(int const robot_id) { return voronoi_cells_[robot_id]; }

  double GetNormalizationFactor() {
    normalization_factor_ = world_idf_ptr_->GetNormalizationFactor();
    return normalization_factor_;
  }

  //! @}
};

} /* namespace CoverageControl */
#endif  // CPPSRC_CORE_INCLUDE_COVERAGECONTROL_COVERAGE_SYSTEM_H_
