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
 * \file parameters.cpp
 * \brief Parses toml configuration file and sets parameters
 */

#include <filesystem>
#include <iostream>

#include "CoverageControl/extern/tomlplusplus/toml.hpp"
#include "CoverageControl/parameters.h"

namespace CoverageControl {
void Parameters::ParseParameters() {
  std::cout << std::boolalpha;
  std::cout << "Using config file: " << config_file_ << std::endl;
  if (not std::filesystem::exists(config_file_)) {
    std::cerr << "Could not find config file " << config_file_ << std::endl;
    throw std::runtime_error("Could not open config file");
  }

  toml::table toml_config;
  try {
    toml_config = toml::parse_file(config_file_);
  } catch (const std::exception& e) {
    std::cerr << "Error parsing config file: " << e.what() << std::endl;
    std::cerr << "Please check the config file format" << std::endl;
    std::cerr << "File: " << config_file_ << std::endl;
    throw std::runtime_error("Error parsing config file");
  }

  if (toml_config["NumRobots"].value<int>()) {
    if (toml_config["NumRobots"].value<int>().value() < 1) {
      std::cerr << "NumRobots must be greater than 0" << std::endl;
      throw std::runtime_error("NumRobots must be greater than 0");
    }
    pNumRobots = toml_config["NumRobots"].value<int>().value();
  } else {
    std::cout << "NumRobots (default): " << pNumRobots << std::endl;
  }

  auto toml_IO = toml_config["IO"];
  if (toml_IO["PlotScale"].value<double>()) {
    pPlotScale = toml_IO["PlotScale"].value<double>().value();
  } else {
    std::cout << "PlotScale (default): " << pPlotScale << std::endl;
  }

  auto toml_EnvironmentMaps = toml_config["Environment"]["Maps"];

  if (toml_EnvironmentMaps) {
    auto toml_Resolution = toml_EnvironmentMaps["Resolution"].value<double>();
    auto toml_WorldMapSize = toml_EnvironmentMaps["WorldMapSize"].value<int>();
    auto toml_RobotMapSize = toml_EnvironmentMaps["RobotMapSize"].value<int>();
    auto toml_LocalMapSize = toml_EnvironmentMaps["LocalMapSize"].value<int>();

    if (toml_Resolution) {
      pResolution = toml_Resolution.value();
    }

    if (toml_WorldMapSize) {
      pWorldMapSize = toml_WorldMapSize.value();
    }
    if (toml_RobotMapSize) {
      pRobotMapSize = toml_RobotMapSize.value();
    }
    if (toml_LocalMapSize) {
      pLocalMapSize = toml_LocalMapSize.value();
    }

    auto toml_EnvironmentMapsUpdateSettings =
        toml_EnvironmentMaps["UpdateSettings"];

    if (toml_EnvironmentMapsUpdateSettings) {
      auto toml_UpdateRobotMap =
          toml_EnvironmentMapsUpdateSettings["UpdateRobotMap"].value<bool>();
      auto toml_UpdateSensorView =
          toml_EnvironmentMapsUpdateSettings["UpdateSensorView"].value<bool>();
      auto toml_UpdateExplorationMap =
          toml_EnvironmentMapsUpdateSettings["UpdateExplorationMap"]
              .value<bool>();
      auto toml_UpdateSystemMap =
          toml_EnvironmentMapsUpdateSettings["UpdateSystemMap"].value<bool>();

      if (toml_UpdateRobotMap) {
        pUpdateRobotMap = toml_UpdateRobotMap.value();
      }
      if (toml_UpdateSensorView) {
        pUpdateSensorView = toml_UpdateSensorView.value();
      }
      if (toml_UpdateExplorationMap) {
        pUpdateExplorationMap = toml_UpdateExplorationMap.value();
      }
      if (toml_UpdateSystemMap) {
        pUpdateSystemMap = toml_UpdateSystemMap.value();
      }
    }
  }

  auto toml_EnvironmentIDF = toml_config["Environment"]["IDF"];

  if (toml_EnvironmentIDF) {
    if (toml_EnvironmentIDF["NumGaussianFeatures"].value<int>()) {
      pNumGaussianFeatures = toml_EnvironmentIDF["NumGaussianFeatures"].value<int>().value();
    } else {
      std::cout << "NumGaussianFeatures (default): " << pNumGaussianFeatures << std::endl;
    }
    auto toml_TruncationBND =
        toml_EnvironmentIDF["TruncationBND"].value<double>();
    auto toml_Norm = toml_EnvironmentIDF["Norm"].value<double>();
    auto toml_MinSigma = toml_EnvironmentIDF["MinSigma"].value<double>();
    auto toml_MaxSigma = toml_EnvironmentIDF["MaxSigma"].value<double>();
    auto toml_MinPeak = toml_EnvironmentIDF["MinPeak"].value<double>();
    auto toml_MaxPeak = toml_EnvironmentIDF["MaxPeak"].value<double>();
    auto toml_UnknownImportance =
        toml_EnvironmentIDF["UnknownImportance"].value<double>();
    auto toml_RobotMapUseUnknownImportance =
        toml_EnvironmentIDF["RobotMapUseUnknownImportance"].value<bool>();

    if (toml_TruncationBND) {
      pTruncationBND = toml_TruncationBND.value();
    }
    if (toml_Norm) {
      pNorm = toml_Norm.value();
    }
    if (toml_MinSigma) {
      pMinSigma = toml_MinSigma.value();
    }
    if (toml_MaxSigma) {
      pMaxSigma = toml_MaxSigma.value();
    }
    if (toml_MinPeak) {
      pMinPeak = toml_MinPeak.value();
    }
    if (toml_MaxPeak) {
      pMaxPeak = toml_MaxPeak.value();
    }

    if (toml_EnvironmentIDF["NumPolygons"].value<int>()) {
      pNumPolygons = toml_EnvironmentIDF["NumPolygons"].value<int>().value();
    } else {
      std::cout << "NumPolygons (default): " << pNumPolygons << std::endl;
    }

    if (toml_EnvironmentIDF["MaxVertices"].value<int>()) {
      pMaxVertices = toml_EnvironmentIDF["MaxVertices"].value<int>().value();
    } else {
      std::cout << "MaxVertices (default): " << pMaxVertices << std::endl;
    }

    if (toml_EnvironmentIDF["PolygonRadius"].value<double>()) {
      pPolygonRadius = toml_EnvironmentIDF["PolygonRadius"].value<double>().value();
    } else {
      std::cout << "PolygonRadius (default): " << pPolygonRadius << std::endl;
    }

    if (toml_UnknownImportance) {
      pUnknownImportance = toml_UnknownImportance.value();
    }
    if (toml_RobotMapUseUnknownImportance) {
      pRobotMapUseUnknownImportance = toml_RobotMapUseUnknownImportance.value();
    }
  }

  auto toml_RobotModel = toml_config["RobotModel"];

  if (toml_RobotModel) {
    auto toml_SensorSize = toml_RobotModel["SensorSize"].value<int>();
    auto toml_CommunicationRange =
        toml_RobotModel["CommunicationRange"].value<double>();
    auto toml_MaxRobotSpeed = toml_RobotModel["MaxRobotSpeed"].value<double>();
    auto toml_RobotInitDist = toml_RobotModel["RobotInitDist"].value<double>();
    auto toml_RobotPosHistorySize =
        toml_RobotModel["RobotPosHistorySize"].value<int>();
    auto toml_TimeStep = toml_RobotModel["TimeStep"].value<double>();

    if (toml_SensorSize) {
      pSensorSize = toml_SensorSize.value();
    }
    if (toml_CommunicationRange) {
      pCommunicationRange = toml_CommunicationRange.value();
    }
    if (toml_MaxRobotSpeed) {
      pMaxRobotSpeed = toml_MaxRobotSpeed.value();
    }
    if (toml_RobotInitDist) {
      pRobotInitDist = toml_RobotInitDist.value();
    }
    if (toml_RobotPosHistorySize) {
      pRobotPosHistorySize = toml_RobotPosHistorySize.value();
    }
    if (toml_TimeStep) {
      pTimeStep = toml_TimeStep.value();
    }
  }

  if (toml_RobotModel["AddNoise"]) {
    auto toml_AddNoisePositions =
        toml_RobotModel["AddNoise"]["AddNoisePositions"].value<bool>();
    auto toml_PositionsNoiseSigma =
        toml_RobotModel["AddNoise"]["PositionsNoiseSigma"].value<double>();
    if (toml_AddNoisePositions) {
      pAddNoisePositions = toml_AddNoisePositions.value();
    }
    if (toml_PositionsNoiseSigma) {
      pPositionsNoiseSigma = toml_PositionsNoiseSigma.value();
    }
  }

  auto toml_Algorithm = toml_config["Algorithm"];

  if (toml_Algorithm) {
    auto toml_EpisodeSteps = toml_Algorithm["EpisodeSteps"].value<int>();
    if (toml_EpisodeSteps) {
      pEpisodeSteps = toml_EpisodeSteps.value();
    }
    if (toml_Algorithm["CheckOscillations"].value<bool>()) {
      pCheckOscillations =
          toml_Algorithm["CheckOscillations"].value<bool>().value();
    }

    auto toml_LloydMaxIterations =
        toml_Algorithm["Global-CVT"]["LloydMaxIterations"].value<int>();
    auto toml_LloydNumTries =
        toml_Algorithm["Global-CVT"]["LloydNumTries"].value<int>();
    if (toml_LloydMaxIterations) {
      pLloydMaxIterations = toml_LloydMaxIterations.value();
    }
    if (toml_LloydNumTries) {
      pLloydNumTries = toml_LloydNumTries.value();
    }

    auto toml_NumFrontiers =
        toml_Algorithm["Exploration"]["NumFrontiers"].value<int>();
    if (toml_NumFrontiers) {
      pNumFrontiers = toml_NumFrontiers.value();
    }
  }
}

void Parameters::PrintParameters() const {
  std::cout << "NumRobots: " << pNumRobots << std::endl;
  std::cout << "NumPolygons: " << pNumPolygons << std::endl;
  std::cout << "MaxVertices: " << pMaxVertices << std::endl;
  std::cout << "PolygonRadius: " << pPolygonRadius << std::endl;

  std::cout << "PlotScale: " << pPlotScale << std::endl;

  std::cout << "Resolution: " << pResolution << std::endl;
  std::cout << "WorldMapSize: " << pWorldMapSize << std::endl;
  std::cout << "RobotMapSize: " << pRobotMapSize << std::endl;
  std::cout << "LocalMapSize: " << pLocalMapSize << std::endl;

  std::cout << "UpdateRobotMap: " << pUpdateRobotMap << std::endl;
  std::cout << "UpdateSensorView: " << pUpdateSensorView << std::endl;
  std::cout << "UpdateExplorationMap: " << pUpdateExplorationMap << std::endl;
  std::cout << "UpdateSystemMap: " << pUpdateSystemMap << std::endl;

  std::cout << "NumGaussianFeatures: " << pNumGaussianFeatures << std::endl;
  std::cout << "TruncationBND: " << pTruncationBND << std::endl;
  std::cout << "Norm: " << pNorm << std::endl;
  std::cout << "MinSigma: " << pMinSigma << std::endl;
  std::cout << "MaxSigma: " << pMaxSigma << std::endl;
  std::cout << "MinPeak: " << pMinPeak << std::endl;
  std::cout << "MaxPeak: " << pMaxPeak << std::endl;

  std::cout << "pNumPolygons: " << pNumPolygons << std::endl;
  std::cout << "pMaxVertices: " << pMaxVertices << std::endl;
  std::cout << "pPolygonRadius: " << pPolygonRadius << std::endl;

  std::cout << "UnknownImportance: " << pUnknownImportance << std::endl;
  std::cout << "RobotMapUseUnknownImportance: " << pRobotMapUseUnknownImportance
            << std::endl;

  std::cout << "SensorSize: " << pSensorSize << std::endl;
  std::cout << "CommunicationRange: " << pCommunicationRange << std::endl;
  std::cout << "MaxRobotSpeed: " << pMaxRobotSpeed << std::endl;
  std::cout << "RobotInitDist: " << pRobotInitDist << std::endl;
  std::cout << "RobotPosHistorySize: " << pRobotPosHistorySize << std::endl;
  std::cout << "TimeStep: " << pTimeStep << std::endl;

  std::cout << "AddNoisePositions: " << pAddNoisePositions << std::endl;
  std::cout << "PositionsNoiseSigma: " << pPositionsNoiseSigma << std::endl;

  std::cout << "EpisodeSteps: " << pEpisodeSteps << std::endl;
  std::cout << "CheckOscillations: " << pCheckOscillations << std::endl;
  std::cout << "LloydMaxIterations: " << pLloydMaxIterations << std::endl;
  std::cout << "LloydNumTries: " << pLloydNumTries << std::endl;
  std::cout << "NumFrontiers: " << pNumFrontiers << std::endl;
}
} /* namespace CoverageControl */
