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
#include <Eigen/Dense> // Eigen is used for maps

#include "constants.h"
#include "parameters.h"
#include "typedefs.h"
#include "bivariate_normal_distribution.h"

namespace coveragecontrol {

	class WorldIDF {
		private:
			std::vector <BivariateNormalDistribution> normal_distributions_;
			MapType world_map_;

			// The diagonal points are given as input
			// Returns the total importance in a rectangle by summing up for normal distribution

		public:
			WorldIDF() {
				world_map_ = MapType(pWorldMapSize, pWorldMapSize);
			}

			/** Add Normal distribution to world IDF **/
			void AddNormalDistribution(BivariateNormalDistribution const &distribution) {
				normal_distributions_.push_back(distribution);
			}

			/** Integrate each normal distribution over a rectangle (cell).
			 *  If the cell is far away, the importance is set to 0
			 **/
			double ComputeImportanceRectangle (Point2 const &bottom_left, Point2 const &top_right) const {
				Point2 bottom_right(top_right.x(), bottom_left.y());
				Point2 top_left(Point2(bottom_left.x(), top_right.y()));
				double importance = 0;
				for(auto const &normal_distribution:normal_distributions_) {
					if(normal_distribution.TransformPoint((bottom_left + top_right)/2.).NormSqr() > pTruncationBND * pTruncationBND + pResolution * pResolution) {
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
				for(size_t i = 0; i < pWorldMapSize; ++i) { // Row (x index)
						double x1 = pResolution * i; // Left x-coordinate of pixel
						double x2 = x1 + pResolution; // Right x-coordinate of pixel
					for(size_t j = 0; j < pWorldMapSize; ++j) { // Column (y index)
						double y1 = pResolution * j; // Lower y-coordinate of pixel
						double y2 = y1 + pResolution; // Upper y-coordinate of pixel
						double importance	= ComputeImportanceRectangle(Point2(x1,y1), Point2(x2,y2));
						if(std::abs(importance) < kEps) {
							importance = 0;
						}
						world_map_(i, j) = importance;
					}
				}
			}

			/** Write the world map to a file **/
			int WriteMap(std::string const &file_name) {
				std::ofstream file_obj(file_name);
				file_obj << world_map_;
				file_obj.close();
				return 0;
			}

			MapType GetSensorData(Point2 const &pos, size_t sensor_size) {
				return MapType(0,0);
			}

	};

} /* namespace coveragecontrol */
#endif /* _COVERAGECONTROL_WORLDIDF_H_ */
