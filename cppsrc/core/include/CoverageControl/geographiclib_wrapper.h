/*!
 * This file is part of the CoverageControl library
 * Wrappers for GeographicLib functions (deprecated)
 *
 * TODO:
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

/**
 * Wrappers for GeographicLib functions
 **/

#ifndef COVERAGECONTROL_GEOGRAPHICLIB_WRAPPER_H_
#define COVERAGECONTROL_GEOGRAPHICLIB_WRAPPER_H_

#include <GeographicLib/LocalCartesian.hpp>
#include "typedefs.h"

namespace CoverageControl {

	class GeoLocalTransform {
		private:
			GeographicLib::LocalCartesian geo_obj_;
		public:
			GeoLocalTransform(double const lat, double const lon, double const height) {
				geo_obj_ = GeographicLib::LocalCartesian(lat, lon, height);
			}

			void Reset(double const lat, double const lon, double const height) {
				geo_obj_.Reset(lat, lon, height);
			}

			Point3 Forward(double const lat, double const lon, double const height) const {
				Point3 xyz;
				geo_obj_.Forward(lat, lon, height, xyz[0], xyz[1], xyz[2]);
				return xyz;
			}

			Point3 Reverse(double const x, double const y, double const height) const {
				Point3 lla;
				geo_obj_.Reverse(x, y, height, lla[0], lla[1], lla[2]);
				return lla;
			}

	};

} /* namespace CoverageControl */
#endif /* COVERAGECONTROL_GEOGRAPHICLIB_WRAPPER_H_ */

