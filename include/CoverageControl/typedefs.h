/**
 * Contains typedefs
 **/

#ifndef COVERAGECONTROL_TYPEDEFS_H_
#define COVERAGECONTROL_TYPEDEFS_H_

#include <vector>
#include <Eigen/Dense> // Eigen is used for maps
#include "vec2d.h"

namespace CoverageControl {

	struct Edge {
		double x1, y1, x2, y2;
		Edge(double const x1_in, double const y1_in, double const x2_in, double const y2_in) : x1{x1_in}, y1{y1_in}, x2{x2_in}, y2{y2_in} {}
	};
	typedef Vec2d Point2;
	typedef std::vector<Point2> PointVector;
	typedef Eigen::MatrixXd MapTypeDbl;
	typedef Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> MapType;

} /* namespace CoverageControl */
#endif /* COVERAGECONTROL_TYPEDEFS_H_ */

