/**
 * Wrappers for GeographicLib functions
 **/

#ifndef COVERAGECONTROL_GEOGRAPHICLIB_WRAPPER_H_
#define COVERAGECONTROL_GEOGRAPHICLIB_WRAPPER_H_

#include "typedefs.h"
#include <GeographicLib/LocalCartesian.hpp>

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

