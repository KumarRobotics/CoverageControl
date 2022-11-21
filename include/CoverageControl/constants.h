/**
 * Contains constants
 **/

#ifndef COVERAGECONTROL_CONSTANTS_H_
#define COVERAGECONTROL_CONSTANTS_H_

#include <cmath>

namespace coveragecontrol {

	double const kEps = 1e-10;
	double constexpr kSqrt2 = std::sqrt(2);
	double constexpr kOneBySqrt2 = 1./std::sqrt(2);
	double constexpr kInfD = std::numeric_limits<double>::infinity();

} /* namespace coveragecontrol */
#endif /* COVERAGECONTROL_CONSTANTS_H_ */

