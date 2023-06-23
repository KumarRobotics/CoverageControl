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
#include "cgal/polygon_utils.h"

namespace CoverageControl {

	class WorldIDF {
		private:
			std::vector <BivariateNormalDistribution> normal_distributions_;
			std::vector <PolygonFeature> polygon_features_;
			MapType world_map_;
			Parameters params_;
			double normalization_factor_ = 0;

			// The diagonal points are given as input
			// Returns the total importance in a rectangle by summing up for normal distribution

		public:
			WorldIDF(size_t sz) {
				world_map_ = MapType(sz, sz);
			}

			WorldIDF(Parameters const &params): params_{params}{
				world_map_ = MapType(params_.pWorldMapSize, params_.pWorldMapSize);
			}

			WorldIDF(Parameters const &params, std::string const &file_name): params_{params}{
				world_map_ = MapType(params_.pWorldMapSize, params_.pWorldMapSize);

				// Load Bivariate Normal Distribution from file
				std::ifstream file(file_name);
				if(!file.is_open()) {
					std::cout << "Error: Could not open file " << file_name << std::endl;
					exit(1);
				}
				std::string type;
				while(file >> type) {
					if(type == "BND") {
						double x, y, sigma_x, sigma_y, rho, scale;
						file >> x >> y >> sigma_x >> sigma_y >> rho >> scale;
						AddNormalDistribution(BivariateNormalDistribution(Point2(x, y), Point2(sigma_x, sigma_y), rho, scale));
					}
					else if(type == "CircularBND") {
						double x, y, sigma, scale;
						file >> x >> y >> sigma >> scale;
						AddNormalDistribution(BivariateNormalDistribution(Point2(x, y), sigma, scale));
					}
					else if(type == "Uniform") {
						int num_vertices;
						file >> num_vertices;
						std::vector <Point2> vertices;
						for(int i = 0; i < num_vertices; ++i) {
							double x, y;
							file >> x >> y;
							vertices.push_back(Point2(x, y));
						}
						double importance;
						file >> importance;
						AddUniformDistributionPolygon(PolygonFeature(vertices, importance));
					}
					else {
						std::cout << "Error: Unknown feature type " << type << std::endl;
						exit(1);
					}
				}
			}

			/** Add a uniform distribution over a polygon to world IDF **/
			void AddUniformDistributionPolygon(PolygonFeature const &poly_feature) {
				polygon_features_.push_back(poly_feature);
			}

			/** Add Normal distribution to world IDF **/
			void AddNormalDistribution(BivariateNormalDistribution const &distribution) {
				normal_distributions_.push_back(distribution);
			}

			/** Add Normal distributions to world IDF **/
			void AddNormalDistribution(std::vector <BivariateNormalDistribution> const &dists) {
				normal_distributions_.reserve(normal_distributions_.size() + dists.size());
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
					Point2 mid_point = (bottom_left + top_right)/2.;
					if(normal_distribution.TransformPoint(mid_point).squaredNorm() > params_.pTruncationBND * params_.pTruncationBND + params_.pResolution * params_.pResolution) {
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

			void GenerateMapCuda () {
				GenerateMapCuda((float)params_.pResolution, (float)params_.pTruncationBND, (int)params_.pWorldMapSize);
			}

			void GenerateMapCuda(float const resolution, float const truncation, int const map_size) {

				int num_dists = normal_distributions_.size();
				/* std::cout << "num_dists: " << num_dists << std::endl; */

				/* BND_Cuda *host_dists = (BND_Cuda*) malloc(num_dists * sizeof(BND_Cuda)); */
				BND_Cuda *host_dists = new BND_Cuda[num_dists];

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

				Polygons_Cuda_Host host_polygons;
				for (auto const &poly_feature:polygon_features_) {
					std::vector <PointVector> partition_polys;
					PolygonYMonotonePartition(poly_feature.poly, partition_polys);
					for(auto const &poly:partition_polys) {
						Bounds bounds;
						for(auto const &pt:poly) {
							float x = static_cast<float>(pt.x());
							float y = static_cast<float>(pt.y());
							host_polygons.x.push_back(x);
							host_polygons.y.push_back(y);
							if(x < bounds.xmin) { bounds.xmin = x; }
							if(y < bounds.ymin) { bounds.ymin = y; }
							if(x > bounds.xmax) { bounds.xmax = x; }
							if(y > bounds.ymax) { bounds.ymax = y; }
						}
						host_polygons.imp.push_back(poly_feature.imp);
						host_polygons.sz.push_back(poly.size());
						host_polygons.bounds.push_back(bounds);
					}
				}

				host_polygons.num_pts = host_polygons.x.size();
				host_polygons.num_polygons = host_polygons.imp.size();

				float f_norm = 0;
				generate_world_map_cuda(host_dists, host_polygons, num_dists, map_size, resolution, truncation, params_.pNorm, world_map_.data(), f_norm);
				normalization_factor_ = static_cast<double>(f_norm);
				/* GenerateMap(); */
			}

			/** Write the world map to a file **/
			int WriteWorldMap(std::string const &file_name) const {
				return MapUtils::WriteMap(world_map_, file_name);
			}

			void GetSubWorldMap(Point2 const &pos, int const sensor_size, MapType &submap) const {
				MapUtils::GetSubMap(params_.pResolution, pos, params_.pWorldMapSize, world_map_, sensor_size, submap);
			}

			void PrintMapSize() const {
				std::cout << "World map size: " << world_map_.rows() << " " << world_map_.cols() << std::endl;
			}

			auto GetNormalizationFactor() const { return normalization_factor_; }

			const MapType& GetWorldMap() const { return world_map_; }

			inline int WriteDistributions(std::string const &file_name) const {
				std::ofstream file(file_name);
				if(!file.is_open()) {
					std::cerr << "Could not open file: " << file_name << std::endl;
					return -1;
				}
				file << std::setprecision(kMaxPrecision);
				for(auto const &dist:normal_distributions_) {
					auto sigma = dist.GetSigma();
					if(sigma.x() == sigma.y()) {
						file << "CircularBND" << std::endl;
						file << dist.GetMean().x() << " " << dist.GetMean().y() << " " << sigma.x() << " " << dist.GetScale() << std::endl;
					} else {
						file << "BND" << std::endl;
						file << dist.GetMean().x() << " " << dist.GetMean().y() << " " << dist.GetSigma().x() << " " << dist.GetSigma().y() << " " << dist.GetRho() << " " << dist.GetScale() << std::endl;
					}
				}
				file.close();
				return 0;
			}

			auto GetNumFeatures() const { return normal_distributions_.size() + polygon_features_.size(); }
	};

} /* namespace CoverageControl */
#endif /* COVERAGECONTROL_WORLDIDF_H_ */
