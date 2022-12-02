#ifndef _COVERAGECONTROL_CGAL_CONFIG_H_
#define _COVERAGECONTROL_CGAL_CONFIG_H_

#include <iterator>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Arr_linear_traits_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
// includes for defining the Voronoi diagram adaptor
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Arr_walk_along_line_point_location.h>

// typedefs for defining the adaptor
/* typedef CGAL::Exact_predicates_inexact_constructions_kernel                  K; */
/* typedef CGAL::Delaunay_triangulation_2<K>                                    DT; */
/* typedef CGAL::Delaunay_triangulation_adaptation_traits_2<DT>                 AT; */
/* typedef CGAL::Delaunay_triangulation_caching_degeneracy_removal_policy_2<DT> AP; */
/* typedef CGAL::Voronoi_diagram_2<DT,AT,AP>                                    VD; */
/* typedef CGAL::Delaunay_triangulation_2<K>																		Delaunay_triangulation_2; */

/* typedef K::CGAL_Point2 CGAL_Point2; */
/* typedef K::Iso_rectangle_2 Iso_rectangle_2; */
/* typedef K::Segment_2 Segment_2; */
/* typedef K::Ray_2 Ray_2; */
/* typedef K::Line_2 Line_2; */

typedef CGAL::Exact_predicates_exact_constructions_kernel K;
typedef K::Point_2 CGAL_Point2;
typedef K::Iso_rectangle_2 Iso_rectangle_2;
typedef K::Segment_2 Segment_2;
typedef K::Ray_2 Ray_2;
typedef K::Line_2 Line_2;
typedef CGAL::Delaunay_triangulation_2<K>  Delaunay_triangulation_2;

typedef CGAL::Polygon_2<K> Polygon_2;
typedef CGAL::Polygon_with_holes_2<K> Polygon_with_holes_2;

typedef CGAL::Arr_linear_traits_2<K> Traits_2;
typedef CGAL::Arrangement_2<Traits_2> Arrangement_2;
typedef CGAL::Arr_walk_along_line_point_location<Arrangement_2> CGAL_pl;

/* typedef K::Intersect_2 Intersect_2; */
/* typedef K::Plane_3 Plane_3; */
/* typedef K::Triangle_2 Triangle_2; */
/* typedef Polygon_2::Vertex_const_iterator VertexConstIterator; */
/* typedef Polygon_2::Vertex_const_circulator VertexConstCirculator; */
/* typedef Polygon_2::Vertex_iterator VertexIterator; */
/* typedef Polygon_2::Vertex_circulator VertexCirculator; */
/* typedef Polygon_2::Edge_const_iterator EdgeConstIterator; */
/* typedef Polygon_2::Edge_const_circulator EdgeConstCirculator; */
/* typedef CGAL::Polygon_with_holes_2<K> PolygonWithHoles; */
/* typedef CGAL::Exact_predicates_inexact_constructions_kernel InexactKernel; */

/* typedef CGAL::Aff_transformation_2<Traits_2> Transform2; */
/* typedef CGAL::Simple_cartesian<double>  K_double; */
/* typedef K_double::CGAL_Point2 Point_2_double; */

#endif /* _COVERAGECONTROL_CGAL_CONFIG_H_ */
