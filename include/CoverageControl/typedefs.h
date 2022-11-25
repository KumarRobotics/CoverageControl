/**
 * Contains typedefs
 **/

#ifndef COVERAGECONTROL_TYPEDEFS_H_
#define COVERAGECONTROL_TYPEDEFS_H_

#include <vector>
#include <Eigen/Dense> // Eigen is used for maps
#include "vec2d.h"

namespace CoverageControl {

	typedef Vec2d Point2;
	typedef std::vector<Point2> PointVector;
	typedef Eigen::MatrixXd MapType;

} /* namespace CoverageControl */
#endif /* COVERAGECONTROL_TYPEDEFS_H_ */

