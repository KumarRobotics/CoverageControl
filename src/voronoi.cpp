#include <list>
#include <omp.h>
#include <CoverageControl/voronoi.h>
#include <CoverageControl/cgal_config.h>
#include <CoverageControl/voronoi_cgal.h>

namespace CoverageControl {

	void Voronoi::ComputeMassCentroid(VoronoiCell &vcell) {
		vcell.mass = 0; vcell.obj = 0;
		vcell.centroid = Point2{0,0};
		auto &cell = vcell.cell;
		int n = cell.size();
		/* Polygon_2 cgal_poly; */
		/* for(auto const &p:vcell.cell) { */
		/* 	cgal_poly.push_back(CGAL_Point2(p.x(), p.y())); */
		/* } */

		int left_id = 0;
		int right_id = 0;
		for(int i = 1; i < n; ++i) {
			if(cell[i].x() < cell[left_id].x()) {
				left_id = i;
			}
			if(cell[i].x() > cell[right_id].x()) {
				right_id = i;
			}
		}

		/* std::cout << "Found left vertex" << std::endl; */
		int min_i = std::round(cell[left_id].x()/resolution_);
		min_i = min_i < 0 ? 0 : min_i;

		int max_i = std::round(cell[right_id].x()/resolution_);
		max_i = max_i > map_size_ ? map_size_ : max_i;

		auto next_id = [=](int const id) { return (id + 1)%n; };
		auto prev_id = [=](int const id) { return id == 0 ? (n - 1) : (id - 1); };

		auto cc_pt_id = next_id(left_id); // Counter-clockwise pointer
		auto c_pt_id  = prev_id(left_id); // Clockwise pointer

		/* std::cout << "cc pointers set" << std::endl; */
		for(int i = min_i; i < max_i; ++i) {
			double x = i * resolution_ + resolution_/2.;
			while(true) {
				if(cc_pt_id == left_id) {break;}
				if(cell[cc_pt_id].x() < x) { cc_pt_id = next_id(cc_pt_id); }
				else {break;}
			}
			while(true) {
				if(c_pt_id == left_id) {break;}
				if(cell[c_pt_id].x() < x) { c_pt_id = prev_id(c_pt_id); }
				else {break;}
			}

			if(cell[cc_pt_id].x() < x) { break; }
			if(cell[c_pt_id].x() < x) { break; }

			auto prev_pt = cell[prev_id(cc_pt_id)];
			auto cc_pt = cell[cc_pt_id];
			auto x1 = prev_pt.x();
			auto y1 = prev_pt.y();
			auto x2 = cc_pt.x();
			auto y2 = cc_pt.y();

			if((x2 - x1) < kEps) { throw std::runtime_error{"Unexpected error!"}; }
			auto y_lower = y1 + (x - x1) * (y2 - y1) / (x2 - x1);

			auto next_pt = cell[next_id(c_pt_id)];
			auto c_pt = cell[c_pt_id];
			x1 = next_pt.x();
			y1 = next_pt.y();
			x2 = c_pt.x();
			y2 = c_pt.y();

			if((x2 - x1) < kEps) { throw std::runtime_error{"Unexpected error!"}; }
			auto y_upper = y1 + (x - x1) * (y2 - y1) / (x2 - x1);

			int min_j = std::round(y_lower/resolution_);
			min_j = min_j < 0 ? 0 : min_j;

			int max_j = std::round(y_upper/resolution_);
			max_j = max_j > map_size_ ? map_size_ : max_j;

			for(int j = min_j; j < max_j; ++j) {
				double y = j * resolution_ + resolution_/2.;
				Point2 curr_pt(x, y);
				auto map_val = map_->operator()(i, j);
				vcell.mass += map_val;
				vcell.centroid += curr_pt * map_val;
				vcell.obj += (curr_pt - vcell.site).squaredNorm() * map_val;
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
					vcell.obj += Point2(x - vcell.site.x(), y - vcell.site.y()).squaredNorm() * (*map_)(i, j);
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

		/* std::cout << "d2 begin" << std::endl; */
		Delaunay_triangulation_2 dt2;
		std::vector<CGAL_Point2> CGAL_sites;
		CGAL_sites.reserve(num_sites_);
		/* std::cout << "Number of sites: " << sites_.size() << std::endl; */
		for(auto const &pt:sites_) {
			CGAL_sites.push_back(CGAL_Point2(pt.x(), pt.y()));
		}
		dt2.insert(CGAL_sites.begin(), CGAL_sites.end());

		CGAL_DelaunayHelper vor;
		dt2.draw_dual(vor);
		/* std::cout << "d2 end" << std::endl; */

		vor.segments_.push_back(Segment_2(CGAL_Point2(0,0), CGAL_Point2(map_size_,0)));
		vor.segments_.push_back(Segment_2(CGAL_Point2(map_size_,0), CGAL_Point2(map_size_, map_size_)));
		vor.segments_.push_back(Segment_2(CGAL_Point2(map_size_, map_size_), CGAL_Point2(0, map_size_)));
		vor.segments_.push_back(Segment_2(CGAL_Point2(0, map_size_), CGAL_Point2(0, 0)));

		Arrangement_2 arr;
		CGAL::insert(arr, vor.rays_.begin(), vor.rays_.end());
		CGAL::insert(arr, vor.lines_.begin(), vor.lines_.end());
		CGAL::insert(arr, vor.segments_.begin(), vor.segments_.end());
		CGAL_pl cgal_pl(arr);
		/* std::cout << "arr end" << std::endl; */

		if(compute_single_ == true) {
			auto pt = CGAL_sites[robot_id_];
			Polygon_2 polygon;
			auto obj = cgal_pl.locate(pt);
			auto f = boost::get<Arrangement_2::Face_const_handle>(&obj);
			CGAL_CCBTraversal<Arrangement_2> ((*f)->outer_ccb(), polygon);
			if(not polygon.is_counterclockwise_oriented()) {
				polygon.reverse_orientation();
			}
			PointVector poly_points;
			for(auto const &p:(polygon)) {
				poly_points.push_back(CGALtoCC(p));
			}
			voronoi_cell_.site = CGALtoCC(pt);
			voronoi_cell_.cell = poly_points;

			ComputeMassCentroid(voronoi_cell_);
			return;
		}

		std::list <Polygon_2> polygon_list;
		CGAL_GeneratePolygons(arr, polygon_list);

		PrunePolygons(polygon_list, map_size_);
		// Create voronoi_cells_ such that the correct cell is assigned to the robot
		/* #pragma omp parallel for */
		for(int iSite = 0; iSite < num_sites_; ++iSite) {
			auto pt = CGAL_sites[iSite];
			Polygon_2 polygon;
			auto obj = cgal_pl.locate(pt);
			auto f = boost::get<Arrangement_2::Face_const_handle>(&obj);
			CGAL_CCBTraversal<Arrangement_2> ((*f)->outer_ccb(), polygon);
			if(not polygon.is_counterclockwise_oriented()) {
				polygon.reverse_orientation();
			}
			PointVector poly_points;
			for(auto const &p:(polygon)) {
				poly_points.push_back(CGALtoCC(p));
			}
			VoronoiCell vcell;
			vcell.site = CGALtoCC(pt);
			vcell.cell = poly_points;
			ComputeMassCentroid(vcell);
			voronoi_cells_[iSite] = vcell;
		}
	}
	/* std::cout << "Voronoi Polygon generated: " << voronoi_cells_.size() << std::endl; */

	// Compute mass and centroid of the cells
	/* #pragma omp parallel for */
	/* 		for(int iCell = 0; iCell < num_sites_; ++iCell) { */
	/* 			auto &vcell = voronoi_cells_[iCell]; */
	/* 			ComputeMassCentroid(vcell); */
	/* 		} */

} /* namespace CoverageControl */
