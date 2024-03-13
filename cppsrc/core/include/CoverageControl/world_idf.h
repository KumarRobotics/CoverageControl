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
#include "CoverageControl/cuda_utils.h"
#ifdef COVERAGECONTROL_WITH_CUDA
#include "CoverageControl/generate_world_map.h"
#endif
#include "CoverageControl/bivariate_normal_distribution.h"
#include "CoverageControl/cgal/polygon_utils.h"
#include "CoverageControl/constants.h"
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
  double normalization_factor_ = 0;
  bool is_cuda_available_ = false;

  /*! Fills in values of the world_map_ with the total importance for each cell
   */
  void GenerateMapCPU() {
    /* std::cout << "Generating map using CPU" << std::endl; */
    float max_importance = 0;
    for (int i = 0; i < params_.pWorldMapSize; ++i) {  // Row (x index)
      float x1 = params_.pResolution * i;   // Left x-coordinate of pixel
      float x2 = x1 + params_.pResolution;  // Right x-coordinate of pixel
      for (int j = 0; j < params_.pWorldMapSize; ++j) {  // Column (y index)
        float y1 = params_.pResolution * j;   // Lower y-coordinate of pixel
        float y2 = y1 + params_.pResolution;  // Upper y-coordinate of pixel
        float importance =
            ComputeImportanceBND(Point2(x1, y1), Point2(x2, y2));
        if (std::abs(importance) < kEps) {
          importance = 0;
        }
        world_map_(i, j) = importance;
        if (importance > max_importance) {
          max_importance = importance;
        }
      }
    }

    if (max_importance < kEps) {
      normalization_factor_ = params_.pNorm;
    } else {
      normalization_factor_ = params_.pNorm / max_importance;
    }

    // Normalize the world map
    for (int i = 0; i < params_.pWorldMapSize; ++i) {
      for (int j = 0; j < params_.pWorldMapSize; ++j) {
        world_map_(i, j) *= normalization_factor_;
      }
    }
  }

  /*! Integrate each normal distribution over a rectangle (cell).
   *  If the cell is far away, the importance is set to 0
   */
  float ComputeImportanceBND(Point2 const &bottom_left,
                                   Point2 const &top_right) const {
    Point2 bottom_right(top_right.x(), bottom_left.y());
    Point2 top_left(Point2(bottom_left.x(), top_right.y()));
    float importance = 0;
    for (auto const &normal_distribution : normal_distributions_) {
      Point2 mid_point = (bottom_left + top_right) / 2.;
      if (normal_distribution.TransformPoint(mid_point).squaredNorm() >
          params_.pTruncationBND * params_.pTruncationBND) {
        continue;
      }
      importance += normal_distribution.IntegrateQuarterPlaneF(bottom_left);
      importance -= normal_distribution.IntegrateQuarterPlaneF(bottom_right);
      importance -= normal_distribution.IntegrateQuarterPlaneF(top_left);
      importance += normal_distribution.IntegrateQuarterPlaneF(top_right);
    }
    return importance;
  }

#ifdef COVERAGECONTROL_WITH_CUDA
  void GenerateMapCuda() {
    /* std::cout << "Generating map using CUDA" << std::endl; */
    GenerateMapCuda(static_cast<float>(params_.pResolution),
                    static_cast<float>(params_.pTruncationBND),
                    static_cast<int>(params_.pWorldMapSize));
  }

  void GenerateMapCuda(float const resolution, float const truncation,
                       int const map_size) {
    int num_dists = normal_distributions_.size();
    /* std::cout << "num_dists: " << num_dists << std::endl; */

    /* BND_Cuda *host_dists = (BND_Cuda*) malloc(num_dists * sizeof(BND_Cuda));
     */
    BND_Cuda *host_dists = new BND_Cuda[num_dists];

    for (int i = 0; i < num_dists; ++i) {
      auto mean = normal_distributions_[i].GetMean();
      host_dists[i].mean_x = static_cast<float>(mean.x());
      host_dists[i].mean_y = static_cast<float>(mean.y());
      auto sigma = normal_distributions_[i].GetSigma();
      host_dists[i].sigma_x = static_cast<float>(sigma.x());
      host_dists[i].sigma_y = static_cast<float>(sigma.y());
      host_dists[i].rho = static_cast<float>(normal_distributions_[i].GetRho());
      host_dists[i].scale =
          static_cast<float>(normal_distributions_[i].GetScale());
    }

    Polygons_Cuda_Host host_polygons;
    for (auto const &poly_feature : polygon_features_) {
      std::vector<PointVector> partition_polys;
      PolygonYMonotonePartition(poly_feature.poly, partition_polys);
      for (auto const &poly : partition_polys) {
        Bounds bounds;
        for (auto const &pt : poly) {
          float x = static_cast<float>(pt.x());
          float y = static_cast<float>(pt.y());
          host_polygons.x.push_back(x);
          host_polygons.y.push_back(y);
          if (x < bounds.xmin) {
            bounds.xmin = x;
          }
          if (y < bounds.ymin) {
            bounds.ymin = y;
          }
          if (x > bounds.xmax) {
            bounds.xmax = x;
          }
          if (y > bounds.ymax) {
            bounds.ymax = y;
          }
        }
        host_polygons.imp.push_back(poly_feature.imp);
        host_polygons.sz.push_back(poly.size());
        host_polygons.bounds.push_back(bounds);
      }
    }

    host_polygons.num_pts = host_polygons.x.size();
    host_polygons.num_polygons = host_polygons.imp.size();

    float f_norm = 0;
    generate_world_map_cuda(host_dists, host_polygons, num_dists, map_size,
                            resolution, truncation, params_.pNorm,
                            world_map_.data(), f_norm);
    normalization_factor_ = static_cast<double>(f_norm);
    /* GenerateMap(); */
  }
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
    GenerateMap();
  }

  WorldIDF(Parameters const &params, std::string const &file_name)
      : WorldIDF(params) {
    // Load Bivariate Normal Distribution from file
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
        std::vector<Point2> vertices;
        for (int i = 0; i < num_vertices; ++i) {
          double x, y;
          file >> x >> y;
          vertices.push_back(Point2(x, y));
        }
        double importance;
        file >> importance;
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

  inline int WriteDistributions(std::string const &file_name) const {
    std::ofstream file(file_name);
    if (!file.is_open()) {
      std::cerr << "Could not open file: " << file_name << std::endl;
      return -1;
    }
    file << std::setprecision(kMaxPrecision);
    for (auto const &dist : normal_distributions_) {
      auto sigma = dist.GetSigma();
      if (sigma.x() == sigma.y()) {
        file << "CircularBND" << std::endl;
        file << dist.GetMean().x() << " " << dist.GetMean().y() << " "
             << sigma.x() << " " << dist.GetScale() << std::endl;
      } else {
        file << "BND" << std::endl;
        file << dist.GetMean().x() << " " << dist.GetMean().y() << " "
             << dist.GetSigma().x() << " " << dist.GetSigma().y() << " "
             << dist.GetRho() << " " << dist.GetScale() << std::endl;
      }
    }
    file.close();
    return 0;
  }

  auto GetNumFeatures() const {
    return normal_distributions_.size() + polygon_features_.size();
  }
};

} /* namespace CoverageControl */
#endif  // CPPSRC_CORE_INCLUDE_COVERAGECONTROL_WORLD_IDF_H_
