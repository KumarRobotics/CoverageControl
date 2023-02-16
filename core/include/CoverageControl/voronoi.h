/**
 * Contains class for computing Voronoi cells
 **/

#ifndef COVERAGECONTROL_VORONOI_H_
#define COVERAGECONTROL_VORONOI_H_

#include <cmath>
#include <utility>
#include <iostream>
#include <memory>
#include "constants.h"
#include "typedefs.h"

namespace CoverageControl {

	struct VoronoiCell {
		Point2 site;
		PointVector cell;
		double mass = 0;
		Point2 centroid;
		double sum_idf_site_dist_sqr = 0;
		double sum_idf_goal_dist_sqr = 0;
		double sum_idf_site_dist = 0;
		double sum_idf_goal_dist = 0;

		void MassCentroidFunctional(double const &map_val, Point2 const &pt) {
			mass += map_val;
			centroid += pt * map_val;
			sum_idf_site_dist += (pt - site).norm() * map_val;
			sum_idf_site_dist_sqr += (pt - site).squaredNorm() * map_val;
		}
		void GoalObjFunctional(double const &map_val, Point2 const &pt) {
			sum_idf_goal_dist_sqr += (pt - centroid).squaredNorm() * map_val;
			sum_idf_goal_dist += (pt - centroid).norm() * map_val;
		}
	};

	class Voronoi {
		private:
			PointVector sites_;
			std::shared_ptr <const MapType> map_ = nullptr;
			Point2 map_size_;
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
			void MassCentroidFunctional(VoronoiCell &vcell, double const &map_val, Point2 const &pt);
			void CellNavigator(VoronoiCell const &, std::function<void (double, Point2 )>);

			/* std::vector <Edge> voronoi_edges_; */
		public:
			Voronoi() {}
			Voronoi(
					PointVector const &sites,
					MapType const &map,
					Point2 map_size,
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
			auto GetVoronoiCells() const {return voronoi_cells_;}
			auto GetVoronoiCell() {return voronoi_cell_;}

			double GetSumIDFSiteDistSqr() {
				double obj = 0;
				for(auto const &cell:voronoi_cells_) {
					obj = obj + cell.sum_idf_site_dist_sqr;
				}
				return obj;
			}
			double GetSumIDFGoalDistSqr() {
				double obj = 0;
				for(auto const &cell:voronoi_cells_) {
					obj = obj + cell.sum_idf_goal_dist_sqr;
				}
				return obj;
			}
			/* auto GetVoronoiEdges() {return voronoi_edges_;} */
	};

} /* namespace CoverageControl */
#endif /* COVERAGECONTROL_VORONOI_H_ */

