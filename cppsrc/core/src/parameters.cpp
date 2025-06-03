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
#include <stdexcept>
#include <string>

#include "CoverageControl/extern/tomlplusplus/toml.hpp"
#include "CoverageControl/parameters.h"

namespace CoverageControl {
namespace {

template <typename T>
bool SafeExtractValue(const toml::node_view<toml::node>& node, T& target,
                      const std::string& param_name = "",
                      bool log_default = true) {
  if (auto val = node.value<T>()) {
    target = val.value();
    return true;
  }
  if (log_default && !param_name.empty()) {
    std::cout << param_name << " (default): " << target << std::endl;
  }
  return false;
}

bool ValidatePositiveInt(int value, const std::string& param_name) {
  if (value <= 0) {
    std::cerr << param_name << " must be greater than 0, got: " << value
              << std::endl;
    return false;
  }
  return true;
}

bool ValidatePositiveDouble(double value, const std::string& param_name) {
  if (value <= 0.0) {
    std::cerr << param_name << " must be greater than 0.0, got: " << value
              << std::endl;
    return false;
  }
  return true;
}

bool ValidateRange(double value, double min, double max,
                   const std::string& param_name) {
  if (value < min || value > max) {
    std::cerr << param_name << " must be between " << min << " and " << max
              << ", got: " << value << std::endl;
    return false;
  }
  return true;
}
}  // namespace

void Parameters::ParseParameters() {
  std::cout << std::boolalpha;
  // std::cout << "Using config file: " << config_file_ << std::endl;
  if (not std::filesystem::exists(config_file_)) {
    throw std::runtime_error("Config file does not exist: " + config_file_);
  }

  toml::table config;
  try {
    config = toml::parse_file(config_file_);
  } catch (const toml::parse_error& e) {
    std::cerr << "TOML parse error in " << config_file_ << ":\n"
              << "  Line " << e.source().begin.line << ", Column "
              << e.source().begin.column << ": " << e.description()
              << std::endl;
    throw std::runtime_error("Failed to parse TOML config file");
  } catch (const std::exception& e) {
    std::cerr << "Error reading config file " << config_file_ << ": "
              << e.what() << std::endl;
    throw std::runtime_error("Failed to read config file");
  }

  try {
    if (SafeExtractValue(config["NumRobots"], pNumRobots, "NumRobots")) {
      if (!ValidatePositiveInt(pNumRobots, "NumRobots")) {
        throw std::runtime_error("Invalid NumRobots value");
      }
    }

    if (auto env_maps = config["Environment"]["Maps"]) {
      SafeExtractValue(env_maps["Resolution"], pResolution, "Resolution");
      SafeExtractValue(env_maps["WorldMapSize"], pWorldMapSize, "WorldMapSize");
      SafeExtractValue(env_maps["RobotMapSize"], pRobotMapSize, "RobotMapSize");
      SafeExtractValue(env_maps["LocalMapSize"], pLocalMapSize, "LocalMapSize");

      // UpdateSettings subsection
      if (auto update_settings = env_maps["UpdateSettings"]) {
        SafeExtractValue(update_settings["UpdateRobotMap"], pUpdateRobotMap,
                         "UpdateRobotMap");
        SafeExtractValue(update_settings["UpdateSensorView"], pUpdateSensorView,
                         "UpdateSensorView");
        SafeExtractValue(update_settings["UpdateExplorationMap"],
                         pUpdateExplorationMap, "UpdateExplorationMap");
        SafeExtractValue(update_settings["UpdateSystemMap"], pUpdateSystemMap,
                         "UpdateSystemMap");
      }
    }

    // Environment.IDF section
    if (auto env_idf = config["Environment"]["IDF"]) {
      SafeExtractValue(env_idf["NumGaussianFeatures"], pNumGaussianFeatures,
                       "NumGaussianFeatures");
      SafeExtractValue(env_idf["TruncationBND"], pTruncationBND,
                       "TruncationBND");
      SafeExtractValue(env_idf["Norm"], pNorm, "Norm");
      SafeExtractValue(env_idf["MinSigma"], pMinSigma, "MinSigma");
      SafeExtractValue(env_idf["MaxSigma"], pMaxSigma, "MaxSigma");
      SafeExtractValue(env_idf["MinPeak"], pMinPeak, "MinPeak");
      SafeExtractValue(env_idf["MaxPeak"], pMaxPeak, "MaxPeak");
      SafeExtractValue(env_idf["NumPolygons"], pNumPolygons, "NumPolygons");
      SafeExtractValue(env_idf["MaxVertices"], pMaxVertices, "MaxVertices");
      SafeExtractValue(env_idf["PolygonRadius"], pPolygonRadius,
                       "PolygonRadius");
      SafeExtractValue(env_idf["UnknownImportance"], pUnknownImportance,
                       "UnknownImportance");
      SafeExtractValue(env_idf["RobotMapUseUnknownImportance"],
                       pRobotMapUseUnknownImportance,
                       "RobotMapUseUnknownImportance");
    }

    if (auto io_section = config["IO"]) {
      SafeExtractValue(io_section["PlotScale"], pPlotScale, "PlotScale");
    }
    if (auto robot_model = config["RobotModel"]) {
      SafeExtractValue(robot_model["SensorSize"], pSensorSize, "SensorSize");
      SafeExtractValue(robot_model["CommunicationRange"], pCommunicationRange,
                       "CommunicationRange");
      SafeExtractValue(robot_model["MaxRobotSpeed"], pMaxRobotSpeed,
                       "MaxRobotSpeed");
      SafeExtractValue(robot_model["RobotInitDist"], pRobotInitDist,
                       "RobotInitDist");
      SafeExtractValue(robot_model["RobotPosHistorySize"], pRobotPosHistorySize,
                       "RobotPosHistorySize");
      SafeExtractValue(robot_model["TimeStep"], pTimeStep, "TimeStep");

      // AddNoise subsection
      if (auto add_noise = robot_model["AddNoise"]) {
        SafeExtractValue(add_noise["AddNoisePositions"], pAddNoisePositions,
                         "AddNoisePositions");
        SafeExtractValue(add_noise["PositionsNoiseSigmaMin"],
                         pPositionsNoiseSigmaMin, "PositionsNoiseSigmaMin");
        SafeExtractValue(add_noise["PositionsNoiseSigmaMax"],
                         pPositionsNoiseSigmaMax, "PositionsNoiseSigmaMax");
      }
    }
    if (auto algorithm = config["Algorithm"]) {
      SafeExtractValue(algorithm["EpisodeSteps"], pEpisodeSteps,
                       "EpisodeSteps");
      SafeExtractValue(algorithm["CheckOscillations"], pCheckOscillations,
                       "CheckOscillations");

      // Global-CVT subsection
      if (auto global_cvt = algorithm["Global-CVT"]) {
        SafeExtractValue(global_cvt["LloydMaxIterations"], pLloydMaxIterations,
                         "LloydMaxIterations");
        SafeExtractValue(global_cvt["LloydNumTries"], pLloydNumTries,
                         "LloydNumTries");
      }

      // Exploration subsection
      if (auto exploration = algorithm["Exploration"]) {
        SafeExtractValue(exploration["NumFrontiers"], pNumFrontiers,
                         "NumFrontiers");
      }
    }

  } catch (const std::exception& e) {
    std::cerr << "Error parsing parameters: " << e.what() << std::endl;
    throw;
  }
  ValidateParameters();
}

void Parameters::ValidateParameters() {
  std::vector<std::string> errors;

  // Validate positive integers
  if (!ValidatePositiveInt(pNumRobots, "NumRobots"))
    errors.push_back("NumRobots");
  if (!ValidatePositiveInt(pWorldMapSize, "WorldMapSize"))
    errors.push_back("WorldMapSize");
  if (!ValidatePositiveInt(pRobotMapSize, "RobotMapSize"))
    errors.push_back("RobotMapSize");
  if (!ValidatePositiveInt(pLocalMapSize, "LocalMapSize"))
    errors.push_back("LocalMapSize");
  if (!ValidatePositiveInt(pSensorSize, "SensorSize"))
    errors.push_back("SensorSize");
  if (!ValidatePositiveInt(pEpisodeSteps, "EpisodeSteps"))
    errors.push_back("EpisodeSteps");

  // Validate positive doubles
  if (!ValidatePositiveDouble(pResolution, "Resolution"))
    errors.push_back("Resolution");
  if (!ValidatePositiveDouble(pCommunicationRange, "CommunicationRange"))
    errors.push_back("CommunicationRange");
  if (!ValidatePositiveDouble(pMaxRobotSpeed, "MaxRobotSpeed"))
    errors.push_back("MaxRobotSpeed");
  if (!ValidatePositiveDouble(pTimeStep, "TimeStep"))
    errors.push_back("TimeStep");

  // Validate ranges
  if (!ValidateRange(pPlotScale, 0.1, 10.0, "PlotScale"))
    errors.push_back("PlotScale");

  // Validate sensor size is even
  if (pSensorSize % 2 != 0) {
    std::cerr << "SensorSize must be even, got: " << pSensorSize << std::endl;
    errors.push_back("SensorSize");
  }

  // Validate sigma ranges
  if (pMinSigma >= pMaxSigma) {
    std::cerr << "MinSigma (" << pMinSigma << ") must be less than MaxSigma ("
              << pMaxSigma << ")" << std::endl;
    errors.push_back("Sigma range");
  }

  // Validate peak ranges
  if (pMinPeak >= pMaxPeak) {
    std::cerr << "MinPeak (" << pMinPeak << ") must be less than MaxPeak ("
              << pMaxPeak << ")" << std::endl;
    errors.push_back("Peak range");
  }

  // Validate noise sigma ranges
  if (pAddNoisePositions &&
      pPositionsNoiseSigmaMin >= pPositionsNoiseSigmaMax) {
    std::cerr << "PositionsNoiseSigmaMin (" << pPositionsNoiseSigmaMin
              << ") must be less than PositionsNoiseSigmaMax ("
              << pPositionsNoiseSigmaMax << ")" << std::endl;
    errors.push_back("Noise sigma range");
  }

  // Validate speed constraint
  double max_displacement_per_step = pMaxRobotSpeed * pTimeStep / pResolution;
  if (max_displacement_per_step >= pSensorSize / 2.0) {
    std::cerr << "Robot speed constraint violated: MaxRobotSpeed * TimeStep / "
                 "Resolution ("
              << max_displacement_per_step << ") must be < SensorSize/2 ("
              << pSensorSize / 2.0 << ")" << std::endl;
    errors.push_back("Speed constraint");
  }
}

void Parameters::PrintParameters() const {
  std::cout << "\n=== CoverageControl Parameters ===" << std::endl;

  std::cout << "\n[Environment]" << std::endl;
  std::cout << "NumRobots: " << pNumRobots << std::endl;
  std::cout << "NumGaussianFeatures: " << pNumGaussianFeatures << std::endl;
  std::cout << "NumPolygons: " << pNumPolygons << std::endl;
  std::cout << "MaxVertices: " << pMaxVertices << std::endl;
  std::cout << "PolygonRadius: " << pPolygonRadius << std::endl;

  std::cout << "\n[IO]" << std::endl;
  std::cout << "PlotScale: " << pPlotScale << std::endl;

  std::cout << "\n[Maps]" << std::endl;
  std::cout << "Resolution: " << pResolution << std::endl;
  std::cout << "WorldMapSize: " << pWorldMapSize << std::endl;
  std::cout << "RobotMapSize: " << pRobotMapSize << std::endl;
  std::cout << "LocalMapSize: " << pLocalMapSize << std::endl;
  std::cout << "UpdateRobotMap: " << pUpdateRobotMap << std::endl;
  std::cout << "UpdateSensorView: " << pUpdateSensorView << std::endl;
  std::cout << "UpdateExplorationMap: " << pUpdateExplorationMap << std::endl;
  std::cout << "UpdateSystemMap: " << pUpdateSystemMap << std::endl;

  std::cout << "\n[IDF Parameters]" << std::endl;
  std::cout << "TruncationBND: " << pTruncationBND << std::endl;
  std::cout << "Norm: " << pNorm << std::endl;
  std::cout << "MinSigma: " << pMinSigma << std::endl;
  std::cout << "MaxSigma: " << pMaxSigma << std::endl;
  std::cout << "MinPeak: " << pMinPeak << std::endl;
  std::cout << "MaxPeak: " << pMaxPeak << std::endl;
  std::cout << "UnknownImportance: " << pUnknownImportance << std::endl;
  std::cout << "RobotMapUseUnknownImportance: " << pRobotMapUseUnknownImportance
            << std::endl;

  std::cout << "\n[Robot Model]" << std::endl;
  std::cout << "SensorSize: " << pSensorSize << std::endl;
  std::cout << "CommunicationRange: " << pCommunicationRange << std::endl;
  std::cout << "MaxRobotSpeed: " << pMaxRobotSpeed << std::endl;
  std::cout << "RobotInitDist: " << pRobotInitDist << std::endl;
  std::cout << "RobotPosHistorySize: " << pRobotPosHistorySize << std::endl;
  std::cout << "TimeStep: " << pTimeStep << std::endl;

  std::cout << "\n[Noise Parameters]" << std::endl;
  std::cout << "AddNoisePositions: " << pAddNoisePositions << std::endl;
  std::cout << "PositionsNoiseSigmaMin: " << pPositionsNoiseSigmaMin
            << std::endl;
  std::cout << "PositionsNoiseSigmaMax: " << pPositionsNoiseSigmaMax
            << std::endl;

  std::cout << "\n[Algorithm]" << std::endl;
  std::cout << "EpisodeSteps: " << pEpisodeSteps << std::endl;
  std::cout << "CheckOscillations: " << pCheckOscillations << std::endl;
  std::cout << "LloydMaxIterations: " << pLloydMaxIterations << std::endl;
  std::cout << "LloydNumTries: " << pLloydNumTries << std::endl;
  std::cout << "NumFrontiers: " << pNumFrontiers << std::endl;

  std::cout << "\n=================================" << std::endl;
}

}  // namespace CoverageControl
