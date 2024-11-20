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
 * \file parameters.h
 * \brief Contains parameters.
 *
 * These are const and variable names start with lower-case p and use CamelCase
 * Primarily used by the RobotModel class, see robot_model.h
 */

#ifndef CPPSRC_CORE_INCLUDE_COVERAGECONTROL_PARAMETERS_H_
#define CPPSRC_CORE_INCLUDE_COVERAGECONTROL_PARAMETERS_H_

#include <cmath>
#include <string>

namespace CoverageControl {

/*!
 * \addtogroup cpp_api
 * @{
 * \class Parameters
 * \brief Class to store parameters
 * \details The parameters are for the robot model and the environment.
 * The parameters are set during runtime through configuration files.
 */
class Parameters {
 public:
  std::string config_file_;

  /*!
   * \name Environment Parameters
   * @{
   */
  int pNumRobots = 32;    //!< Number of robots

  /*! \name IO Parameters
   * @{
   */
  //! Determines the quality of the plot and videos
  //! > 1: High quality (takes more time to plot)
  //! < 1: Low quality (takes less time to plot)
  double pPlotScale = 1.0;
  /*! @} */

  /*! \name Map Parameters
   * @{
   */
  //! Assuming same resolution in both the directions in meters. Pixel area =
  //! pResolution^2
  double pResolution = 1.0;

  //! Actual size of maps is size * pResolution, e.g.,  pWorldMapSize *
  //! pResolution
  int pWorldMapSize = 1024;

  //! Robot map saves what the robot has seen
  int pRobotMapSize = pWorldMapSize;

  //! Local map is used for computing mass.
  //! Actual area would be pLocalMapSize * pResolution
  //! \warning Should be greater than pCommunicationRange so that they can form
  //! different channels of the same image.
  int pLocalMapSize = 256;

  bool pUpdateRobotMap = true;
  bool pUpdateExplorationMap = true;
  bool pUpdateSensorView = true;
  bool pUpdateSystemMap = true;

  /*! @} */

  /*! \name IDF Parameters
   * @{
   */

  //! Bivariate Normal Distribution truncated after pTruncationBND * sigma
  // Helps in reducing the number of erfc evaluations
  // Needs testing to be sure that the probability masses are not significantly
  // off
  int pNumGaussianFeatures = 32;  //!< Number of features
  double pTruncationBND = 2;

  double pNorm = 1;

  // These settings are only required if the IDF is generated using random
  // gaussians
  double pMinSigma = 40;
  double pMaxSigma = 50;
  double pMinPeak = 6;
  double pMaxPeak = 10;

  int pNumPolygons = 0;   //!< Number of polygonal features
  int pMaxVertices = 10;  //!< Maximum number of vertices in a polygon
  double pPolygonRadius = 64;

  double pUnknownImportance = 0.5;
  bool pRobotMapUseUnknownImportance = false;
  /*! @} */
  /*! @} */

  /*! \name Robot Model Parameters
   * @{
   */

  // Assuming square sensor FOV.
  // Actual FOV: square with side pResolution * pSensorSize
  // Robot is placed at the center of FOV
  // Make it even so that I don't have to deal with substracting by
  // half-resolution. Have made it to half of (pWorldMapSize - 1000 /
  // pResolution)/2
  int pSensorSize = 64;  //! \warning Positive integer. NOTE: Needs to be even
  double pCommunicationRange = 128;  //!< Radius of communication (in meters)
  // in m/s. Make sure pMaxRobotSpeed * pTimeStep / pResolution < pSensorSize/2
  double pMaxRobotSpeed = 5;
  double pRobotInitDist = 1024;   //!< Distance from the origin within which to
                                  //!< initialize the position of the robots
  int pRobotPosHistorySize = 20;  //!< Number of previous positions to store
  double pTimeStep = 1;  //!< Each time step corresponds to pTimeStep seconds

  /*! \name Add Noise
   * @{
   */
  bool pAddNoisePositions = false;
  double pPositionsNoiseSigma = 0.;
  /*! @} */
  /*! @} */

  /*! \name Algorithm Parameters
   * @{
   */

  int pEpisodeSteps = 2000;  // Total time is pEpisodeSteps * pTimeStep
  bool pCheckOscillations = true;

  /*! \name Global-CVT
   * @{
   */
  int pLloydMaxIterations = 100;
  int pLloydNumTries = 10;
  /*! @} */

  /*! \name Exploration
   * @{
   */
  int pNumFrontiers = 10;  // Number of frontiers to be selected
                           /*! @} */

  /*! @} */
  Parameters() {}

  explicit Parameters(std::string const &config_file)
      : config_file_{config_file} {
    ParseParameters();
    /* PrintParameters(); */
  }

  void SetConfig(std::string const &config_file) {
    config_file_ = config_file;
    ParseParameters();
  }

  void PrintParameters() const;

 private:
  void ParseParameters();
};

/*!
 * @}
 */

} /* namespace CoverageControl */

#endif  // CPPSRC_CORE_INCLUDE_COVERAGECONTROL_PARAMETERS_H_
