/*!
 * This file is part of the CoverageControl library
 *
 * Provides utilities for polygon manipulation using CGAL.
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
#ifndef COVERAGECONTROl_CGAL_POLYGON_UTILS_H_
#define COVERAGECONTROl_CGAL_POLYGON_UTILS_H_

#include "../typedefs.h"
#include <vector>

namespace CoverageControl {

	/*! \brief Partition a polygon into y-monotone polygons
	 *
	 * @param[in] polygon The input polygon
	 * @param[out] y_monotone_polygons The output y-monotone polygons
	 */
void PolygonYMonotonePartition(PointVector const &, std::vector <PointVector> &);

} /* namespace CoverageControl */

#endif /* COVERAGECONTROl_CGAL_POLYGON_UTILS_H_ */
