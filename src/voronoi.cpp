#include <list>
#include <omp.h>
#include <CoverageControl/voronoi.h>
#include <CoverageControl/cgal_config.h>
#include <CoverageControl/voronoi_cgal.h>

namespace CoverageControl {

	void Voronoi::ComputeMassCentroid(VoronoiCell &vcell) {
		/* std::cout << "Entering ComputeMassCentroid " << std::endl; */
		vcell.mass = 0;
		vcell.obj = 0;
		vcell.centroid = Point2(0,0);
		Polygon_2 cgal_poly;
		for(auto const &p:vcell.cell) {
			cgal_poly.push_back(CGAL_Point2(p.x(), p.y()));
		}

		auto start_vertex_itr = cgal_poly.vertices_circulator();
		auto curr_vertex_itr = start_vertex_itr;
		std::advance(curr_vertex_itr, 1);
		auto left_vertex_itr = start_vertex_itr;
		while(curr_vertex_itr != start_vertex_itr) {
			if(curr_vertex_itr->x() < left_vertex_itr->x()) {
				left_vertex_itr = curr_vertex_itr;
			}
			std::advance(curr_vertex_itr, 1);
		}

		/* std::cout << "Found left vertex" << std::endl; */
		int min_i = std::round(CGAL::to_double(left_vertex_itr->x())/resolution_);
		min_i = min_i < 0 ? 0 : min_i;

		auto right_vertex_itr = cgal_poly.right_vertex();
		int max_i = std::round(CGAL::to_double(right_vertex_itr->x())/resolution_);
		max_i = max_i > map_size_ ? map_size_ : max_i;

		auto cc_pt_itr = std::next(left_vertex_itr); // Counter-clockwise pointer
		auto c_pt_itr = std::prev(left_vertex_itr); // Clockwise pointer

		/* std::cout << "cc pointers set" << std::endl; */
		for(int i = min_i; i < max_i; ++i) {
			double x = i * resolution_ + resolution_/2.;
			while(true) {
				if(cc_pt_itr == left_vertex_itr) {break;}
				if(CGAL::to_double(cc_pt_itr->x()) < x) { std::advance(cc_pt_itr, 1); }
				else {break;}
			}
			while(true) {
				if(c_pt_itr == left_vertex_itr) {break;}
				if(CGAL::to_double(c_pt_itr->x()) < x) { std::advance(c_pt_itr, -1); }
				else {break;}
			}
			if(CGAL::to_double(c_pt_itr->x()) < x) { break; }
			if(CGAL::to_double(cc_pt_itr->x()) < x) { break; }

			auto x1 = std::prev(cc_pt_itr)->x();
			auto y1 = std::prev(cc_pt_itr)->y();
			auto x2 = cc_pt_itr->x();
			auto y2 = cc_pt_itr->y();
			if((x2 - x1) < kEps) { throw std::runtime_error{"Unexpected error!"}; }
			auto y_lower = y1 + (x - x1) * (y2 - y1) / (x2 - x1);
			x1 = std::next(c_pt_itr)->x();
			y1 = std::next(c_pt_itr)->y();
			x2 = c_pt_itr->x();
			y2 = c_pt_itr->y();
			if((x2 - x1) < kEps) { throw std::runtime_error{"Unexpected error!"}; }
			auto y_upper = y1 + (x - x1) * (y2 - y1) / (x2 - x1);

			int min_j = std::round(CGAL::to_double(y_lower)/resolution_);
			min_j = min_j < 0 ? 0 : min_j;

			int max_j = std::round(CGAL::to_double(y_upper)/resolution_);
			max_j = max_j > map_size_ ? map_size_ : max_j;

			for(int j = min_j; j < max_j; ++j) {
				double y = j * resolution_ + resolution_/2.;
				auto map_val = map_->operator()(i, j);
				vcell.mass += map_val;
				vcell.centroid = vcell.centroid + Point2(x, y) * map_val;
				vcell.obj += Point2(x - vcell.site.x(), y - vcell.site.y()).NormSqr() * map_val;
			}
		}
		/* std::cout << "Computed vcell" << std::endl; */
		if(vcell.mass < kEps) {
			vcell.centroid = vcell.site;
		} else {
			vcell.centroid = vcell.centroid / vcell.mass;
		}
		/* std::cout << "Exiting ComputeMassCentroid " << std::endl; */
	}

	// This is an older slightly conservative approach.
	// It is less prone to errors as it uses existing functions from the library.
	// Will have to stress test the above version.
	// Initial tests seem to all pass
	void Voronoi::ComputeMassCentroid2(VoronoiCell &vcell) {
		vcell.mass = 0;
		vcell.obj = 0;
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
					vcell.obj += Point2(x - vcell.site.x(), y - vcell.site.y()).NormSqr() * (*map_)(i, j);
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
		if(num_sites_ == 1) {
			VoronoiCell vcell;
			vcell.site = sites_[0];
			double sz = map_size_;
			vcell.cell = PointVector{Point2{0,0}, Point2{sz,0}, Point2{sz, sz}, Point2{0,sz}};
			ComputeMassCentroid(vcell);
			if(compute_single_ == true) {
				voronoi_cell_ = vcell;
			} else {
				voronoi_cells_.push_back(vcell);
			}
			return;
		}

		Delaunay_triangulation_2 dt2;
		std::vector<CGAL_Point2> CGAL_sites;
		CGAL_sites.reserve(num_sites_);
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
		Arrangement_2 arr;
		CGAL::insert(arr, vor.rays_.begin(), vor.rays_.end());
		CGAL::insert(arr, vor.lines_.begin(), vor.lines_.end());
		CGAL::insert(arr, vor.segments_.begin(), vor.segments_.end());
		/* std::cout << "Arrangement created" << std::endl; */

		std::list <Polygon_2> polygon_list;
		CGAL_GeneratePolygons(arr, polygon_list);
		/* std::cout << "Polygons generated" << std::endl; */

		if(compute_single_ == true) {
			/* std::cout << " Inside if" << std::endl; */
			auto const &pt = CGAL_sites[robot_id_];
			/* std::cout << pt << std::endl; */
			/* std::cout << "List size: " << polygon_list.size() << std::endl; */
			for(auto it = polygon_list.begin(); it != polygon_list.end(); ++it) {
				if(IsPointInPoly(pt, *it) == true) {
					PointVector poly_points;
					poly_points.reserve(it->size());
					for(auto const &p:(*it)) {
						poly_points.push_back(CGALtoCC(p));
					}
					voronoi_cell_.site = CGALtoCC(pt);
					voronoi_cell_.cell = poly_points;
					break;
				}
			}
			/* std::cout << "Computing mass" << std::endl; */
			ComputeMassCentroid(voronoi_cell_);
			std::cout << "Mass: " << voronoi_cell_.mass << std::endl;
			/* std::cout << "If ends" << std::endl; */
			return;
		}
		PrunePolygons(polygon_list, map_size_);
		// Create voronoi_cells_ such that the correct cell is assigned to the robot
		for(int iSite = 0; iSite < num_sites_; ++iSite) {
			auto const &pt = CGAL_sites[iSite];
			for(auto it = polygon_list.begin(); it != polygon_list.end(); ++it) {
				if(IsPointInPoly(pt, *it) == true) {
					PointVector poly_points;
					poly_points.reserve(it->size());
					for(auto const &p:(*it)) {
						poly_points.push_back(CGALtoCC(p));
					}
					VoronoiCell vcell;
					vcell.site = CGALtoCC(pt);
					vcell.cell = poly_points;
					voronoi_cells_[iSite] = vcell;
					it = polygon_list.erase(it);
					break;
				}
			}
		}
		/* std::cout << "Voronoi Polygon generated: " << voronoi_cells_.size() << std::endl; */

		// Compute mass and centroid of the cells
		#pragma omp parallel for
		for(int iCell = 0; iCell < num_sites_; ++iCell) {
			auto &vcell = voronoi_cells_[iCell];
			ComputeMassCentroid(vcell);
			/* ComputeMassCentroid2(vcell); */
		}
	}

} /* namespace CoverageControl */
