#include <CoverageControl/voronoi.h>
#include <CoverageControl/cgal_config.h>

namespace CoverageControl {

	Point2 CGALtoCC(Point_2 const pt) {
		return Point2(pt.x(), pt.y());
	}
	struct CGAL_Cropped_voronoi_from_delaunay{
		std::list<Segment_2> m_cropped_vd;
		Iso_rectangle_2 bbox_;
		std::vector<Ray_2> rays_;
		std::vector<Line_2> lines_;
		std::vector<Segment_2> segments_;
		CGAL_Cropped_voronoi_from_delaunay(Iso_rectangle_2 const &bbox):bbox_(bbox) {}
		template <class RSL>
			void crop_and_extract_segment(const RSL& rsl){
				CGAL::Object obj = CGAL::intersection(rsl, bbox_);
				const Segment_2* s=CGAL::object_cast<Segment_2>(&obj);
				if (s) m_cropped_vd.push_back(*s);
			}
		void operator<<(const Ray_2& ray)    { crop_and_extract_segment(ray); rays_.push_back(ray); }
		void operator<<(const Line_2& line)  { crop_and_extract_segment(line); lines_.push_back(line); }
		void operator<<(const Segment_2& seg){ crop_and_extract_segment(seg); segments_.push_back(seg); }
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
		void CGAL_GeneratePolygons (const Arrangement& arr, std::vector <Polygon_2> &polygon_list) {
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

	void Voronoi::ComputeVoronoiCells() {
		Delaunay_triangulation_2 dt2;
		std::vector<Point_2> CGAL_sites;
		for(auto const pt:sites_) {
			CGAL_sites.push_back(Point_2(pt.x(), pt.y()));
		}
		dt2.insert(CGAL_sites.begin(), CGAL_sites.end());

		Iso_rectangle_2 bbox(0, 0, map_size_, map_size_);
		CGAL_Cropped_voronoi_from_delaunay vor(bbox);
		dt2.draw_dual(vor);

		Arrangement_2 arr_;
		CGAL::insert(arr_, vor.rays_.begin(), vor.rays_.end());
		CGAL::insert(arr_, vor.lines_.begin(), vor.lines_.end());
		CGAL::insert(arr_, vor.segments_.begin(), vor.segments_.end());
		auto s1 = Segment_2(Point_2(0,0), Point_2(map_size_,0));
		CGAL::insert(arr_, s1);
		auto s2 = Segment_2(Point_2(map_size_,0), Point_2(map_size_, map_size_));
		CGAL::insert(arr_, s2);
		auto s3 = Segment_2(Point_2(map_size_, map_size_), Point_2(0, map_size_));
		CGAL::insert(arr_, s3);
		auto s4 = Segment_2(Point_2(0, map_size_), Point_2(0, 0));
		CGAL::insert(arr_, s4);

		std::vector <Polygon_2> polygon_list;
		CGAL_GeneratePolygons(arr_, polygon_list);
		for(auto const &poly:polygon_list) {
			PointVector poly_points;
			for(auto const &p:poly) {
				poly_points.push_back(Point2(p.x(), p.y()));
			}
			voronoi_cells_.push_back(poly_points);
		}

		for(auto it = vor.m_cropped_vd.begin(); it != vor.m_cropped_vd.end(); ++it) {
			auto source = CGALtoCC(it->source());
			auto target = CGALtoCC(it->target());
			voronoi_edges_.push_back(std::make_pair(source, target));
		}
	}

} /* namespace CoverageControl */
