#include <list>
#include <omp.h>
#include <CoverageControl/voronoi.h>
#include <CoverageControl/cgal_config.h>
#include <CoverageControl/voronoi_cgal.h>

namespace CoverageControl {

	void Voronoi::ComputeMassCentroid(VoronoiCell &vcell) {
			vcell.mass = 0;
			vcell.centroid = Point2(0,0);
			Polygon_2 cgal_poly;
			for(auto const &p:vcell.cell) {
				cgal_poly.push_back(CGAL_Point2(p.x(), p.y()));
			}
			int min_i = std::floor(CGAL::to_double(cgal_poly.left_vertex()->x())/resolution_);
			min_i = min_i < 0 ? 0 : min_i;
			int max_i = std::ceil(CGAL::to_double(cgal_poly.right_vertex()->x())/resolution_);
			max_i = max_i > map_size_ ? map_size_ : max_i;
			int min_j = std::floor(CGAL::to_double(cgal_poly.bottom_vertex()->y())/resolution_);
			min_j = min_j < 0 ? 0 : min_j;
			int max_j = std::ceil(CGAL::to_double(cgal_poly.top_vertex()->y())/resolution_);
			max_j = max_j > map_size_ ? map_size_ : max_j;

			for(int i = min_i; i < max_i; ++i) {
				for(int j = min_j; j < max_j; ++j) {
					double x = i * resolution_ + resolution_/2.;
					double y = j * resolution_ + resolution_/2.;
					CGAL_Point2 pt(x, y);
					if(CGAL::bounded_side_2(cgal_poly.begin(), cgal_poly.end(), pt, K()) == CGAL::ON_UNBOUNDED_SIDE) {
						continue;
					} else {
						vcell.mass += (*map_)(i, j);
						vcell.centroid = vcell.centroid + Point2(x, y) * (*map_)(i, j);
					}
				}
			}
			if(vcell.mass < kEps) {
				vcell.centroid = vcell.site;
			} else {
				vcell.centroid = vcell.centroid / vcell.mass;
			}
	}

	void Voronoi::ComputeVoronoiCells() {
		Delaunay_triangulation_2 dt2;
		std::vector<CGAL_Point2> CGAL_sites;
		/* std::cout << "Number of sites: " << sites_.size() << std::endl; */
		for(auto const pt:sites_) {
			CGAL_sites.push_back(CGAL_Point2(pt.x(), pt.y()));
		}
		dt2.insert(CGAL_sites.begin(), CGAL_sites.end());

		CGAL_Cropped_voronoi_from_delaunay vor;
		dt2.draw_dual(vor);

		auto s1 = Segment_2(CGAL_Point2(0,0), CGAL_Point2(map_size_,0));
		vor.segments_.push_back(s1);
		auto s2 = Segment_2(CGAL_Point2(map_size_,0), CGAL_Point2(map_size_, map_size_));
		vor.segments_.push_back(s2);
		auto s3 = Segment_2(CGAL_Point2(map_size_, map_size_), CGAL_Point2(0, map_size_));
		vor.segments_.push_back(s3);
		auto s4 = Segment_2(CGAL_Point2(0, map_size_), CGAL_Point2(0, 0));
		vor.segments_.push_back(s4);

		/* std::cout << "Computing arrangement" << std::endl; */
		Arrangement_2 arr_;
		CGAL::insert(arr_, vor.rays_.begin(), vor.rays_.end());
		CGAL::insert(arr_, vor.lines_.begin(), vor.lines_.end());
		CGAL::insert(arr_, vor.segments_.begin(), vor.segments_.end());
		/* std::cout << "Arrangement created" << std::endl; */

		std::list <Polygon_2> polygon_list;
		CGAL_GeneratePolygons(arr_, polygon_list);
		/* std::cout << "Polygons generated" << std::endl; */

		if(compute_single_ == true) {
			auto &pt = CGAL_sites[robot_id_];
			for(auto it = polygon_list.begin(); it != polygon_list.end(); ++it) {
				if(IsPointInPoly(pt, *it) == true) {
					PointVector poly_points;
					for(auto const &p:(*it)) {
						poly_points.push_back(CGALtoCC(p));
					}
					voronoi_cell_.site = CGALtoCC(pt);
					voronoi_cell_.cell = poly_points;
					break;
				}
			}
			ComputeMassCentroid(voronoi_cells_[robot_id_]);
		} else {
			PrunePolygons(polygon_list, map_size_);

			// Create voronoi_cells_ such that the correct cell is assigned to the robot
			voronoi_cells_.reserve(num_robots_);
			for(auto const &pt:CGAL_sites) {
				for(auto it = polygon_list.begin(); it != polygon_list.end(); ++it) {
					if(IsPointInPoly(pt, *it) == true) {
						PointVector poly_points;
						for(auto const &p:(*it)) {
							poly_points.push_back(CGALtoCC(p));
						}
						VoronoiCell vcell;
						vcell.site = CGALtoCC(pt);
						vcell.cell = poly_points;
						voronoi_cells_.push_back(vcell);
						it = polygon_list.erase(it);
						break;
					}
				}
			}
			/* std::cout << "Voronoi Polygon generated: " << voronoi_cells_.size() << std::endl; */

			// Compute mass and centroid of the cells
#pragma omp parallel for
			for(int iCell = 0; iCell < num_robots_; ++iCell) {
				auto &vcell = voronoi_cells_[iCell];
				ComputeMassCentroid(vcell);
			}
		}

	}

} /* namespace CoverageControl */
