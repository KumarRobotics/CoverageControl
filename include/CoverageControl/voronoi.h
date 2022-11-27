/**
 * Contains class for computing Voronoi cells
 **/

#ifndef _COVERAGECONTROL_VORONOI_H_
#define _COVERAGECONTROL_VORONOI_H_

#include <cmath>
#include <utility>
#include <iostream>
#include <CoverageControl/typedefs.h>

namespace CoverageControl {

	class Voronoi {
		private:
			PointVector sites_;
			int map_size_;
			std::vector <PointVector> voronoi_cells_;
			Point2 centroid_;
			std::vector <std::pair<Point2, Point2>> voronoi_edges_;
			double mass;
		public:
			Voronoi() {}
			Voronoi(PointVector const &sites, int const map_size) : sites_{sites}, map_size_{map_size} {
				std::cout << " Contructor called voronoi" << std::endl;
				ComputeVoronoiCells();
			}
			void ComputeVoronoiCells();
			void ComputeCentroid(MapType const &map);
			Point2 GetCentroid() { return centroid_; }
			double GetMass() { return mass; }
			std::vector<PointVector> GetVoronoiCells() {return voronoi_cells_;}
	};

} /* namespace CoverageControl */
#endif /* _COVERAGECONTROL_VORONOI_H_ */

