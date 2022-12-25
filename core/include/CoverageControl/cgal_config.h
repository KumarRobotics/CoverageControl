#ifndef COVERAGECONTROL_CGAL_CONFIG_H_
#define COVERAGECONTROL_CGAL_CONFIG_H_

#include <iterator>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Arr_linear_traits_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Arr_walk_along_line_point_location.h>

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

#endif /* COVERAGECONTROL_CGAL_CONFIG_H_ */
