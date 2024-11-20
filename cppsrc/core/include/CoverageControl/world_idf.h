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
 * \file world_idf.h
 * \brief Contains the class for Importance Density Function (IDF) for the world
 */

#ifndef CPPSRC_CORE_INCLUDE_COVERAGECONTROL_WORLD_IDF_H_
#define CPPSRC_CORE_INCLUDE_COVERAGECONTROL_WORLD_IDF_H_

#define EIGEN_NO_CUDA   // Don't use eigen's cuda facility
#include <Eigen/Dense>  // Eigen is used for maps
                        //
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "CoverageControl/Config.h"
#include "CoverageControl/bivariate_normal_distribution.h"
#include "CoverageControl/cuda_utils.h"
#include "CoverageControl/map_utils.h"
#include "CoverageControl/parameters.h"
#include "CoverageControl/typedefs.h"

namespace CoverageControl {

/*!
 * \addtogroup cpp_api
 * @{
 * \class WorldIDF
 * @}
 * \brief Class for Importance Density Function (IDF) for the world
 *
 * This class contains the world IDF as a 2D map. The world IDF is computed by
 * integrating the normal distributions over the world. The world IDF is used to
 * compute the importance of each cell in the world. Can use CUDA for faster
 * computation of cell importance.
 */
class WorldIDF {
 private:
  std::vector<BivariateNormalDistribution> normal_distributions_;
  std::vector<PolygonFeature> polygon_features_;
  MapType world_map_;
  Parameters params_;
  float normalization_factor_ = 0;
  bool is_cuda_available_ = false;

  /*! Integrate each normal distribution over a rectangle (cell).
   *  If the cell is far away, the importance is set to 0
   */
  template <class PointType = Point2, typename NumType = double>
  NumType ComputeImportanceBND(PointType const &bottom_left,
                               PointType const &top_right) const {
    PointType bottom_right(top_right.x(), bottom_left.y());
    PointType top_left(bottom_left.x(), top_right.y());
    NumType importance = 0;
    PointType mid_point = (bottom_left + top_right) / 2.f;
    float trun_f = static_cast<float>(params_.pTruncationBND);
    NumType trun_sqr = trun_f * trun_f;
    for (auto const &normal_distribution : normal_distributions_) {
      PointType mid_transformed = normal_distribution.TransformPoint(mid_point);
      if (mid_transformed[0] * mid_transformed[0] +
              mid_transformed[1] * mid_transformed[1] >
          trun_sqr) {
        continue;
      }
      importance += normal_distribution.IntegrateQuarterPlane(bottom_left);
      importance -= normal_distribution.IntegrateQuarterPlane(bottom_right);
      importance -= normal_distribution.IntegrateQuarterPlane(top_left);
      importance += normal_distribution.IntegrateQuarterPlane(top_right);
    }
    return importance;
  }

#ifdef COVERAGECONTROL_WITH_CUDA
  void GenerateMapCuda(float const resolution, float const truncation,
                       int const map_size);
#endif

 public:
  explicit WorldIDF(size_t sz) {
    world_map_ = MapType(sz, sz);
    if (CudaUtils::InitializeCUDA() == true) {
      is_cuda_available_ = true;
    } else {
      is_cuda_available_ = false;
    }
  }

  explicit WorldIDF(Parameters const &params) : WorldIDF(params.pWorldMapSize) {
    params_ = params;
  }

  WorldIDF(Parameters const &params, MapType const &world_map)
      : WorldIDF(params.pWorldMapSize) {
    params_ = params;
    if (world_map.rows() != params.pWorldMapSize ||
        world_map.cols() != params.pWorldMapSize) {
      std::cout << "Error: World map size does not match the specified size"
                << std::endl;
      throw std::invalid_argument(
          "World map size does not match the specified size");
      exit(1);
    }
    world_map_ = world_map;
  }

  WorldIDF(Parameters const &params, std::string const &file_name)
      : WorldIDF(params.pWorldMapSize) {
    params_ = params;
    // Load Bivariate Normal Distribution from file
    params_ = params;
    std::ifstream file(file_name);
    if (!file.is_open()) {
      std::cout << "Error: Could not open file " << file_name << std::endl;
      exit(1);
    }
    std::string type;
    while (file >> type) {
      if (type == "BND") {
        double x, y, sigma_x, sigma_y, rho, scale;
        file >> x >> y >> sigma_x >> sigma_y >> rho >> scale;
        AddNormalDistribution(BivariateNormalDistribution(
            Point2(x, y), Point2(sigma_x, sigma_y), rho, scale));
      } else if (type == "CircularBND") {
        double x, y, sigma, scale;
        file >> x >> y >> sigma >> scale;
        AddNormalDistribution(
            BivariateNormalDistribution(Point2(x, y), sigma, scale));
      } else if (type == "Uniform") {
        int num_vertices;
        file >> num_vertices;
        double importance;
        file >>
            importance;  // importance moved here. Be careful with semantic file
        std::vector<Point2> vertices;
        for (int i = 0; i < num_vertices; ++i) {
          double x, y;
          file >> x >> y;
          vertices.push_back(Point2(x, y));
        }
        AddUniformDistributionPolygon(PolygonFeature(vertices, importance));
      } else {
        std::cout << "Error: Unknown feature type " << type << std::endl;
        exit(1);
      }
    }
    GenerateMap();
  }

  /*! Copy constructor to handle deep copy of the world map */
  WorldIDF(WorldIDF const &other) {
    world_map_ = other.world_map_;
    params_ = other.params_;
    normalization_factor_ = other.normalization_factor_;
    is_cuda_available_ = other.is_cuda_available_;
    normal_distributions_ = other.normal_distributions_;
    polygon_features_ = other.polygon_features_;
  }

  WorldIDF &operator=(WorldIDF const &other) {
    if (this != &other) {
      world_map_ = other.world_map_;
      params_ = other.params_;
      normalization_factor_ = other.normalization_factor_;
      is_cuda_available_ = other.is_cuda_available_;
      normal_distributions_ = other.normal_distributions_;
      polygon_features_ = other.polygon_features_;
    }
    return *this;
  }

  void LoadMap(std::string const &file_name) {
    std::ifstream file(file_name);
    if (!file.is_open()) {
      std::cout << "Error: Could not open file " << file_name << std::endl;
      exit(1);
    }
    for (int i = 0; i < params_.pWorldMapSize; ++i) {
      for (int j = 0; j < params_.pWorldMapSize; ++j) {
        file >> world_map_(i, j);
      }
    }
  }

  /*! Add a uniform distribution over a polygon to world IDF */
  void AddUniformDistributionPolygon(PolygonFeature const &poly_feature) {
    polygon_features_.push_back(poly_feature);
  }

  /*! Add Normal distribution to world IDF */
  void AddNormalDistribution(BivariateNormalDistribution const &distribution) {
    normal_distributions_.push_back(distribution);
  }

  /*! Add Normal distributions to world IDF */
  void AddNormalDistribution(
      std::vector<BivariateNormalDistribution> const &dists) {
    normal_distributions_.reserve(normal_distributions_.size() + dists.size());
    for (auto const &dist : dists) {
      normal_distributions_.push_back(dist);
    }
  }

  void GenerateMap() {
    /* std::cout << "Generating map" << std::endl; */
#ifdef COVERAGECONTROL_WITH_CUDA
    /* GenerateMapCuda(); */
    if (is_cuda_available_) {
      GenerateMapCuda();
    } else {
      GenerateMapCPU();
    }
#else
    GenerateMapCPU();
#endif
  }
  /*! Write the world map to a file */
  int WriteWorldMap(std::string const &file_name) const {
    return MapUtils::WriteMap(world_map_, file_name);
  }

  void GetSubWorldMap(Point2 const &pos, int const sensor_size,
                      MapType &submap) const {
    MapUtils::GetSubMap(params_.pResolution, pos, params_.pWorldMapSize,
                        world_map_, sensor_size, submap);
  }

  void PrintMapSize() const {
    std::cout << "World map size: " << world_map_.rows() << " "
              << world_map_.cols() << std::endl;
  }

  auto GetNormalizationFactor() const { return normalization_factor_; }

  const MapType &GetWorldMap() const { return world_map_; }

  MapType &GetWorldMapMutable() { return world_map_; }

  int WriteDistributions(std::string const &file_name) const;

  auto GetNumFeatures() const {
    return normal_distributions_.size() + polygon_features_.size();
  }

  void GenerateMapCPU();

#ifdef COVERAGECONTROL_WITH_CUDA
  void GenerateMapCuda() {
    /* std::cout << "Generating map using CUDA" << std::endl; */
    GenerateMapCuda(static_cast<float>(params_.pResolution),
                    static_cast<float>(params_.pTruncationBND),
                    static_cast<int>(params_.pWorldMapSize));
  }
#endif
};

} /* namespace CoverageControl */
#endif  // CPPSRC_CORE_INCLUDE_COVERAGECONTROL_WORLD_IDF_H_
