#ifndef COVERAGECONTROl_CGAL_POLYGON_UTILS_H_
#define COVERAGECONTROl_CGAL_POLYGON_UTILS_H_

/* Partition a Polygon into y-motone polygons
 * See https://doc.cgal.org/latest/Partition_2/index.html
 */

#include "../typedefs.h"
#include <vector>

namespace CoverageControl {

void PolygonYMonotonePartition(PointVector const &, std::vector <PointVector> &);

} /* namespace CoverageControl */

#endif /* COVERAGECONTROl_CGAL_POLYGON_UTILS_H_ */
