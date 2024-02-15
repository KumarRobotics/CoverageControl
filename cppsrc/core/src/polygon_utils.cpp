/*!
 * This file is part of the CoverageControl library
 *
 * Helper functions for CGAL operations.
 *
 * @author Saurav Agarwal
 * @contact sauravag@seas.upenn.edu, agr.saurav1@gmail.com
 * Repository: https://github.com/KumarRobotics/CoverageControl
 *
 * The CoverageControl library is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
 *
 * DISCLAIMER OF WARRANTIES: THE SOFTWARE IS PROVIDED "AS-IS" WITHOUT WARRANTY OF ANY KIND INCLUDING ANY WARRANTIES OF PERFORMANCE OR MERCHANTABILITY OR FITNESS FOR A PARTICULAR USE OR PURPOSE OR OF NON-INFRINGEMENT. YOU BEAR ALL RISK RELATING TO QUALITY AND PERFORMANCE OF THE SOFTWARE OR HARDWARE.
 *
 * SUPPORT AND MAINTENANCE: No support, installation, or training is provided.
 *
 * You should have received a copy of the GNU General Public License along with CoverageControl library. If not, see <https://www.gnu.org/licenses/>.
 */

#include "../include/CoverageControl/cgal/polygon_utils.h"
#include "../include/CoverageControl/cgal/config.h"
#include "../include/CoverageControl/cgal/utilities.h"

#include <vector>
#include <list>

namespace CoverageControl {
void PolygonYMonotonePartition(PointVector const &poly, std::vector <PointVector> &new_polys) {

	// Transform general polygon to CGAL //
	Partition_traits_2::Polygon_2 cgal_poly;
	for(auto const &pt:poly) {
		cgal_poly.push_back(Partition_traits_2::Point_2(pt.x(), pt.y()));
	}

	/* std::cout << "Is simple: " << cgal_poly.is_simple() << std::endl; */
	/* std::cout << "Is orientation: " << cgal_poly.orientation() << std::endl; */
	if(cgal_poly.orientation() == CGAL::CLOCKWISE) {
		cgal_poly.reverse_orientation();
	}
	// Obtain partition //
	std::list<Partition_traits_2::Polygon_2> partition_polys;
	CGAL::y_monotone_partition_2(cgal_poly.begin(), cgal_poly.end(), std::back_inserter(partition_polys));

	// Transform to coveragecontrol data type

	new_polys.reserve(new_polys.size() + partition_polys.size()); 
	for(auto const &p:partition_polys) {
		PointVector new_p;
		new_p.reserve(p.size());
    std::transform(p.vertices_begin(), p.vertices_end(), std::back_inserter(new_p),
                   [](CGAL_Point2 const &pt) { return CGALtoCC(pt); });
		new_polys.push_back(new_p);
	}
}
} /* namespace CoverageControl */
