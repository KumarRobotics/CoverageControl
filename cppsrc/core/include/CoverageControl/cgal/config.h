/*
 * This file is part of the CoverageControl library
 *
 * Author: Saurav Agarwal
 * Contact: sauravag@seas.upenn.edu, agr.saurav1@gmail.com
 * Repository: https://github.com/KumarRobotics/CoverageControl
 *
 * Copyright (c) 2024, Saurav Agarwal
 *
 * The CoverageControl library is free software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * The CoverageControl library is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * CoverageControl library. If not, see <https://www.gnu.org/licenses/>.
 */

/*!
 * \file config.h
 * \brief Contains the configuration for the CGAL library.
 */

#ifndef CPPSRC_CORE_INCLUDE_COVERAGECONTROL_CGAL_CONFIG_H_
#define CPPSRC_CORE_INCLUDE_COVERAGECONTROL_CGAL_CONFIG_H_

// #include <CGAL/basic.h>
#include <CGAL/Arr_linear_traits_2.h>
// #include <CGAL/Arr_walk_along_line_point_location.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Partition_traits_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Random.h>
#include <CGAL/algorithm.h>
#include <CGAL/centroid.h>
#include <CGAL/partition_2.h>
#include <CGAL/point_generators_2.h>
#include <CGAL/random_polygon_2.h>
#include <CGAL/Arr_point_location_result.h>
#include <CGAL/Arr_batched_point_location.h>

typedef CGAL::Exact_predicates_exact_constructions_kernel K;
typedef K::Point_2 CGAL_Point2;
typedef K::Iso_rectangle_2 Iso_rectangle_2;
typedef K::Segment_2 Segment_2;
typedef K::Ray_2 Ray_2;
typedef K::Line_2 Line_2;
typedef CGAL::Delaunay_triangulation_2<K> Delaunay_triangulation_2;

typedef CGAL::Polygon_2<K> Polygon_2;
typedef CGAL::Polygon_with_holes_2<K> Polygon_with_holes_2;

typedef CGAL::Arr_linear_traits_2<K> Traits_2;
typedef CGAL::Arrangement_2<Traits_2> Arrangement_2;

typedef CGAL::Partition_traits_2<K> Partition_traits_2;

typedef CGAL::Creator_uniform_2<double, CGAL_Point2> Creator;
typedef CGAL::Random_points_in_square_2<CGAL_Point2, Creator> Point_generator;

// typedef CGAL::Arr_walk_along_line_point_location<Arrangement_2> CGAL_pl;
using CGAL_Point_location_result = CGAL::Arr_point_location_result<Arrangement_2>;
using CGAL_Query_result = std::pair<CGAL_Point2, CGAL_Point_location_result::Type>;

#endif  // CPPSRC_CORE_INCLUDE_COVERAGECONTROL_CGAL_CONFIG_H_
