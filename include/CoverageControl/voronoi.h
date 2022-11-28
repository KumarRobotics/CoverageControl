/**
 * Contains class for computing Voronoi cells
 **/

#ifndef _COVERAGECONTROL_VORONOI_H_
#define _COVERAGECONTROL_VORONOI_H_

#include <cmath>
#include <utility>
#include <iostream>
#include <memory>
#include <CoverageControl/typedefs.h>

namespace CoverageControl {

	struct VoronoiCell {
		PointVector cell;
		double mass = 0;
		Point2 centroid;
		Point2 site;
	};

	class Voronoi {
		private:
			PointVector sites_;
			int map_size_;
			std::vector <VoronoiCell> voronoi_cells_;
			/* std::vector <Edge> voronoi_edges_; */
			std::shared_ptr <const MapType> map_;
			double resolution_ = 0;
		public:
			Voronoi() {}
			Voronoi(PointVector const &sites, MapType const &map, int const map_size, double const &resolution) : sites_{sites}, map_size_{map_size}, resolution_{resolution} {
				map_ = std::make_shared<const MapType>(map);
				ComputeVoronoiCells();
			}
			void ComputeVoronoiCells();
			auto GetVoronoiCells() {return voronoi_cells_;}
			/* auto GetVoronoiEdges() {return voronoi_edges_;} */
	};

} /* namespace CoverageControl */
#endif /* _COVERAGECONTROL_VORONOI_H_ */

