/**
 * Contains class for computing Voronoi cells
 **/

#ifndef _COVERAGECONTROL_VORONOI_H_
#define _COVERAGECONTROL_VORONOI_H_

#include <cmath>
#include <utility>
#include <iostream>
#include <memory>
#include <CoverageControl/constants.h>
#include <CoverageControl/typedefs.h>

namespace CoverageControl {

	struct VoronoiCell {
		Point2 site;
		PointVector cell;
		double mass = 0;
		Point2 centroid;
		double obj = 0;
	};

	class Voronoi {
		private:
			PointVector sites_;
			std::shared_ptr <const MapType> map_ = nullptr;
			int map_size_;
			double resolution_ = 0;

			// compute_single_ is used to determine if the voronoi mass and centroid needs to be computed for only a single site, given by robot_id_. This is useful for distributed voronoi computation
			// The result is stored in voronoi_cell_ (as opposed to voronoi_cells_ when compute_single_ = false)
			bool compute_single_ = false;
			int robot_id_ = 0;
			VoronoiCell voronoi_cell_;

			int num_sites_;
			std::vector <VoronoiCell> voronoi_cells_;
			void ComputeMassCentroid(VoronoiCell &);
			void ComputeMassCentroid2(VoronoiCell &);

			/* std::vector <Edge> voronoi_edges_; */
		public:
			Voronoi() {}
			Voronoi(
					PointVector const &sites,
					MapType const &map,
					int const map_size,
					double const &resolution,
					bool const compute_single = false,
					int const robot_id = 0) :
				sites_{sites},
				map_size_{map_size},
				resolution_{resolution},
				compute_single_{compute_single},
				robot_id_{robot_id} {

				map_ = std::make_shared<const MapType>(map);
				num_sites_ = sites_.size();
				if(compute_single_ == false) {
					voronoi_cells_.resize(num_sites_);
				}
				ComputeVoronoiCells();
			}

			// Update the cites and recompute the voronoi
			void UpdateSites(PointVector const &sites) {
				sites_ = sites;
				num_sites_ = sites_.size();
				ComputeVoronoiCells();
			}

			void ComputeVoronoiCells();
			auto GetVoronoiCells() {return voronoi_cells_;}
			auto GetVoronoiCell() {return voronoi_cell_;}

			double GetObjValue() {
				double obj = 0;
				for(auto const &cell:voronoi_cells_) {
					obj = obj + cell.obj;
				}
				return obj;
			}
			/* auto GetVoronoiEdges() {return voronoi_edges_;} */
	};

} /* namespace CoverageControl */
#endif /* _COVERAGECONTROL_VORONOI_H_ */

