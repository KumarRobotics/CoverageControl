/**
 * Contains constants
 **/

#ifndef COVERAGECONTROL_CONSTANTS_H_
#define COVERAGECONTROL_CONSTANTS_H_

#include <cmath>

namespace CoverageControl {

	double const kEps = 1e-10;
	double const kSqrt2 = std::sqrt(2);
	double const kOneBySqrt2 = 1./std::sqrt(2);
	double const kInfD = std::numeric_limits<double>::infinity();

} /* namespace CoverageControl */
#endif /* COVERAGECONTROL_CONSTANTS_H_ */

