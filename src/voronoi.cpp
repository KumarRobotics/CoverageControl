#include <list>
#include <omp.h>
#include <CoverageControl/voronoi.h>
#include <CoverageControl/cgal_config.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Polygon_2_algorithms.h>

namespace CoverageControl {

	Point2 CGALtoCC(CGAL_Point2 const pt) {
		return Point2(CGAL::to_double(pt.x()), CGAL::to_double(pt.y()));
	}
	struct CGAL_Cropped_voronoi_from_delaunay{
		/* std::list<Segment_2> m_cropped_vd; */
		Iso_rectangle_2 bbox_;
		std::vector<Ray_2> rays_;
		std::vector<Line_2> lines_;
		std::vector<Segment_2> segments_;
		CGAL_Cropped_voronoi_from_delaunay(Iso_rectangle_2 const &bbox):bbox_(bbox) {}
		/* template <class RSL> */
			/* void crop_and_extract_segment(const RSL& rsl){ */
			/* 	CGAL::Object obj = CGAL::intersection(rsl, bbox_); */
			/* 	const Segment_2* s=CGAL::object_cast<Segment_2>(&obj); */
			/* 	if (s) m_cropped_vd.push_back(*s); */
			/* } */
		void operator<<(const Ray_2& ray)    { rays_.push_back(ray); }
		void operator<<(const Line_2& line)  { lines_.push_back(line); }
		void operator<<(const Segment_2& seg){ segments_.push_back(seg); }
	};

	template<class Arrangement>
		void CGAL_CCBTraversal (typename Arrangement::Ccb_halfedge_const_circulator circ, Polygon_2 &polygon) {
			polygon.clear();
			typename Arrangement::Ccb_halfedge_const_circulator curr = circ;
			typename Arrangement::Halfedge_const_handle he;
			auto pt = curr->source()->point();
			do {
				he = curr; pt = he->target()->point();
				polygon.push_back(pt); ++curr;
			} while (curr != circ);
		}

	template<class Arrangement>
		void CGAL_GeneratePolygons (const Arrangement& arr, std::list <Polygon_2> &polygon_list) {
			CGAL_precondition (arr.is_valid());
			typename Arrangement::Face_const_iterator    fit;
			for (fit = arr.faces_begin(); fit != arr.faces_end(); ++fit) {
				if (fit->is_unbounded()) { continue; }
				else { Polygon_2 polygon;
					CGAL_CCBTraversal<Arrangement> (fit->outer_ccb(), polygon);
					polygon_list.push_back(polygon);
				}
			}
		}

	inline bool IsPointInPoly(CGAL_Point2 const &pt, Polygon_2 const &poly) {
		if(CGAL::bounded_side_2(poly.begin(), poly.end(),pt, K()) == CGAL::ON_UNBOUNDED_SIDE) {
			return false;
		}
		return true;
	}

	void Voronoi::ComputeVoronoiCells() {
		Delaunay_triangulation_2 dt2;
		std::vector<CGAL_Point2> CGAL_sites;
		/* std::cout << "Number of sites: " << sites_.size() << std::endl; */
		for(auto const pt:sites_) {
			CGAL_sites.push_back(CGAL_Point2(pt.x(), pt.y()));
		}
		dt2.insert(CGAL_sites.begin(), CGAL_sites.end());

		Iso_rectangle_2 bbox(0, 0, map_size_, map_size_);
		CGAL_Cropped_voronoi_from_delaunay vor(bbox);
		dt2.draw_dual(vor);
		Polygon_2 bbox_poly;
		bbox_poly.push_back(CGAL_Point2(0, 0));
		bbox_poly.push_back(CGAL_Point2(map_size_, 0));
		bbox_poly.push_back(CGAL_Point2(map_size_, map_size_));
		bbox_poly.push_back(CGAL_Point2(0, map_size_));


		auto s1 = Segment_2(CGAL_Point2(0,0), CGAL_Point2(map_size_,0));
		/* CGAL::insert(arr_, s1); */
		vor.segments_.push_back(s1);
		auto s2 = Segment_2(CGAL_Point2(map_size_,0), CGAL_Point2(map_size_, map_size_));
		/* CGAL::insert(arr_, s2); */
		vor.segments_.push_back(s2);
		auto s3 = Segment_2(CGAL_Point2(map_size_, map_size_), CGAL_Point2(0, map_size_));
		/* CGAL::insert(arr_, s3); */
		vor.segments_.push_back(s3);
		auto s4 = Segment_2(CGAL_Point2(0, map_size_), CGAL_Point2(0, 0));
		/* CGAL::insert(arr_, s4); */
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

		for(auto it = polygon_list.begin(); it != polygon_list.end();) {
			if(not CGAL::do_intersect(bbox_poly, *it)) {
				it = polygon_list.erase(it);
			} else {
				++it;
			}
		}
		/* std::cout << "Polygon pruning done" << std::endl; */

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
		/* std::cout << "Arrangement polygon list obtained" << std::endl; */
		/* for(auto &poly:polygon_list) { */
		/* 	if(not CGAL::do_intersect(bbox_poly, poly)) { continue; } */
		/* 	PointVector poly_points; */
		/* 	for(auto const &p:poly) { */
		/* 		poly_points.push_back(CGALtoCC(p)); */
		/* 	} */
		/* 	voronoi_cells_.push_back(poly_points); */
		/* } */

		/* for(auto it = vor.m_cropped_vd.begin(); it != vor.m_cropped_vd.end(); ++it) { */
		/* 	auto source = CGALtoCC(it->source()); */
		/* 	auto target = CGALtoCC(it->target()); */
		/* 	voronoi_edges_.push_back(Edge(source.x(), source.y(), target.x(), target.y())); */
		/* } */
	}

} /* namespace CoverageControl */
