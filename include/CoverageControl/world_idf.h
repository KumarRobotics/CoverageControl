/**
 * A class for Importance Density Function (IDF) for the world
 *
 * TODO: Add functionalities for importance features that are expressed as rectangles, simple polygons, and circles
 **/

#ifndef COVERAGECONTROL_WORLDIDF_H_
#define COVERAGECONTROL_WORLDIDF_H_

#include <vector>
#include <fstream>
#include <iostream>
#define EIGEN_NO_CUDA // Don't use eigen's cuda facility
#include <Eigen/Dense> // Eigen is used for maps

#include "constants.h"
#include "parameters.h"
#include "typedefs.h"
#include "bivariate_normal_distribution.h"
#include "generate_world_map.ch"
#include "map_utils.h"

namespace CoverageControl {

	class WorldIDF {
		private:
			std::vector <BivariateNormalDistribution> normal_distributions_;
			MapType world_map_;
			Parameters params_;
			double normalization_factor_ = 0;

			// The diagonal points are given as input
			// Returns the total importance in a rectangle by summing up for normal distribution

		public:
			WorldIDF(Parameters const &params): params_{params}{
				world_map_ = MapType(params_.pWorldMapSize, params_.pWorldMapSize);
			}

			/** Add Normal distribution to world IDF **/
			void AddNormalDistribution(BivariateNormalDistribution const &distribution) {
				normal_distributions_.push_back(distribution);
			}

			/** Add Normal distributions to world IDF **/
			void AddNormalDistribution(std::vector <BivariateNormalDistribution> const &dists) {
				normal_distributions_.reserve(dists.size());
				for(auto const &dist:dists) {
					normal_distributions_.push_back(dist);
				}
			}

			/** Integrate each normal distribution over a rectangle (cell).
				*  If the cell is far away, the importance is set to 0
				**/
			double ComputeImportanceRectangle (Point2 const &bottom_left, Point2 const &top_right) const {
				Point2 bottom_right(top_right.x(), bottom_left.y());
				Point2 top_left(Point2(bottom_left.x(), top_right.y()));
				double importance = 0;
				for(auto const &normal_distribution:normal_distributions_) {
					if(normal_distribution.TransformPoint((bottom_left + top_right)/2.).NormSqr() > params_.pTruncationBND * params_.pTruncationBND + params_.pResolution * params_.pResolution) {
						continue;
					}
					importance += normal_distribution.IntegrateQuarterPlane(bottom_left);
					importance -= normal_distribution.IntegrateQuarterPlane(bottom_right);
					importance -= normal_distribution.IntegrateQuarterPlane(top_left);
					importance += normal_distribution.IntegrateQuarterPlane(top_right);
				}
				return importance;
			}

			/** Fills in values of the world_map_ with the total importance for each cell **/
			void GenerateMap() {
				for(int i = 0; i < params_.pWorldMapSize; ++i) { // Row (x index)
					double x1 = params_.pResolution * i; // Left x-coordinate of pixel
					double x2 = x1 + params_.pResolution; // Right x-coordinate of pixel
					for(int j = 0; j < params_.pWorldMapSize; ++j) { // Column (y index)
						double y1 = params_.pResolution * j; // Lower y-coordinate of pixel
						double y2 = y1 + params_.pResolution; // Upper y-coordinate of pixel
						double importance	= ComputeImportanceRectangle(Point2(x1,y1), Point2(x2,y2));
						if(std::abs(importance) < kEps) {
							importance = 0;
						}
						world_map_(i, j) = importance;
					}
				}
			}

			void GenerateMapCuda() {
				float resolution = (float) params_.pResolution;
				float truncation = (float) params_.pTruncationBND;
				int map_size = (int) params_.pWorldMapSize;

				int num_dists = normal_distributions_.size();
				/* std::cout << "num_dists: " << num_dists << std::endl; */

				BND_Cuda *host_dists = (BND_Cuda*) malloc(num_dists * sizeof(BND_Cuda));

				for(int i = 0; i < num_dists; ++i) {
					auto mean = normal_distributions_[i].GetMean();
					host_dists[i].mean_x = (float)(mean.x());
					host_dists[i].mean_y = (float)(mean.y());
					auto sigma = normal_distributions_[i].GetSigma();
					host_dists[i].sigma_x = (float)(sigma.x());
					host_dists[i].sigma_y = (float)(sigma.y());
					host_dists[i].rho = (float)(normal_distributions_[i].GetRho());
					host_dists[i].scale = (float)(normal_distributions_[i].GetScale());
				}

				float *importance_vec = (float*) malloc(params_.pWorldMapSize * params_.pWorldMapSize * sizeof(float));
				float max = 0;
				generate_world_map_cuda(host_dists, num_dists, map_size, resolution, truncation, importance_vec, max);
				/* GenerateMap(); */
				normalization_factor_ = params_.pNorm / max;
				for(int i = 0; i < params_.pWorldMapSize; ++i) {
					for(int j = 0; j < params_.pWorldMapSize; ++j) {
						/* if(std::abs(world_map_(i, j) - (double) importance_vec[i*pWorldMapSize + j]) > 10e-5) { */
						/* 	std::cout << "Diff: " << i << " " << j <<  " " << world_map_(i, j) << " " << (double) importance_vec[i*pWorldMapSize + j] << std::endl; */
						/* } */
						/* world_map_(i, j) = (double) (importance_vec[i * params_.pWorldMapSize + j]); */
						/* MapType2 map(params_.pWorldMapSize, params_.pWorldMapSize); */
						world_map_(i, j) = static_cast<unsigned char> (std::round(importance_vec[i * params_.pWorldMapSize + j] * normalization_factor_));
					}
				}

				free(importance_vec);
				free(host_dists);
			}

			/** Write the world map to a file **/
			int WriteWorldMap(std::string const &file_name) const {
				return MapUtils::WriteMap(world_map_, file_name);
			}

			void GetSubWorldMap(Point2 const &pos, int const sensor_size, MapType &submap) const {
				MapUtils::GetSubMap(params_.pResolution, pos, sensor_size, params_.pWorldMapSize, world_map_, submap);
			}

			void PrintMapSize() const {
				std::cout << "World map size: " << world_map_.rows() << " " << world_map_.cols() << std::endl;
			}

			auto GetNormalizationFactor() const { return normalization_factor_; }

			const MapType& GetWorldMap() const { return world_map_; }

	};

} /* namespace CoverageControl */
#endif /* _COVERAGECONTROL_WORLDIDF_H_ */
