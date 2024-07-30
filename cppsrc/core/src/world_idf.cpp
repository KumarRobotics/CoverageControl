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
 * \file world_idf.cpp
 * \brief Contains the implementation of the WorldIDF class.
 */

#include "CoverageControl/cgal/config.h"
#include "CoverageControl/cgal/polygon_utils.h"
#include "CoverageControl/constants.h"
#include "CoverageControl/world_idf.h"
#ifdef COVERAGECONTROL_WITH_CUDA
#include "CoverageControl/generate_world_map.h"
#endif

namespace CoverageControl {

/*! Fills in values of the world_map_ with the total importance for each cell
 */
void WorldIDF::GenerateMapCPU() {
  /* std::cout << "Generating map using CPU" << std::endl; */
  std::vector<Polygon_2> cgal_polygons;
  cgal_polygons.reserve(polygon_features_.size());
  for (auto const &poly : polygon_features_) {
    Polygon_2 cgal_poly;
    for (auto const &pt : poly.poly) {
      cgal_poly.push_back(CGAL_Point2(pt.x(), pt.y()));
    }
    cgal_polygons.push_back(cgal_poly);
  }
  float max_importance = 0;
  float res = static_cast<float>(params_.pResolution);
  for (int i = 0; i < params_.pWorldMapSize; ++i) {  // Row (x index)
    float x1 = res * i;   // Left x-coordinate of pixel
    float x2 = x1 + res;  // Right x-coordinate of pixel
    for (int j = 0; j < params_.pWorldMapSize; ++j) {  // Column (y index)
      float y1 = res * j;   // Lower y-coordinate of pixel
      float y2 = y1 + res;  // Upper y-coordinate of pixel
      Point2f pt1(x1, y1);
      Point2f pt2(x2, y2);
      float importance = ComputeImportanceBND<Point2f, float>(pt1, pt2);
      /* importance += ComputeImportancePoly(pt1, pt2); */
      CGAL_Point2 mid((x1 + x2) / 2, (y1 + y2) / 2);
      float importance_poly = 0;
      for (size_t k = 0; k < cgal_polygons.size(); ++k) {
        auto &poly = cgal_polygons[k];
        if (poly.bounded_side(mid) == CGAL::ON_BOUNDED_SIDE) {
          importance_poly = std::max(importance_poly, polygon_features_[k].imp);
        }
      }
      importance += importance_poly;
      /* auto importance = ComputeImportanceBND(pt1, pt2); */
      /* if (std::abs(importance) < kEps) { */
      /*   importance = 0; */
      /* } */
      world_map_(i, j) = importance;
      if (importance > max_importance) {
        max_importance = importance;
      }
    }
  }

  if (max_importance < kEps) {
    normalization_factor_ = static_cast<float>(params_.pNorm);
  } else {
    normalization_factor_ = static_cast<float>(params_.pNorm) / max_importance;
  }

  if (not(normalization_factor_ > 1e-5)) {  // Parity with CUDA code
    return;
  }

  // Normalize the world map
#pragma omp parallel for
  for (int i = 0; i < params_.pWorldMapSize; ++i) {
    for (int j = 0; j < params_.pWorldMapSize; ++j) {
      world_map_(i, j) *= normalization_factor_;
    }
  }
}

#ifdef COVERAGECONTROL_WITH_CUDA
void WorldIDF::GenerateMapCuda(float const resolution, float const truncation,
                               int const map_size) {
  int num_dists = normal_distributions_.size();
  /* std::cout << "num_dists: " << num_dists << std::endl; */

  /* BND_Cuda *host_dists = (BND_Cuda*) malloc(num_dists * sizeof(BND_Cuda));
   */
  BND_Cuda *host_dists = new BND_Cuda[num_dists];

  for (int i = 0; i < num_dists; ++i) {
    Point2 mean = normal_distributions_[i].GetMean();
    host_dists[i].mean_x = static_cast<float>(mean.x());
    host_dists[i].mean_y = static_cast<float>(mean.y());
    Point2 sigma = normal_distributions_[i].GetSigma();
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
      for (Point2 const &pt : poly) {
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
}
#endif

int WorldIDF::WriteDistributions(std::string const &file_name) const {
  std::ofstream file(file_name);
  if (!file.is_open()) {
    std::cerr << "Could not open file: " << file_name << std::endl;
    return -1;
  }
  file << std::setprecision(kMaxPrecision);
  for (auto const &dist : normal_distributions_) {
    Point2 sigma = dist.GetSigma();
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
  for (auto const &poly : polygon_features_) {
    file << "Uniform" << std::endl;
    file << poly.poly.size() << " " << poly.imp << std::endl;
    for (auto const &pt : poly.poly) {
      file << pt.x() << " " << pt.y() << std::endl;
    }
  }
  file.close();
  return 0;
}
}  // namespace CoverageControl
