/**
 * A struct for Bivariate Normal Distribution
 * The distribution is in it general form
 *
 **/

#ifndef _COVERAGECONTROL_BIVARIATENORMALDISTRIBUTION_H_
#define _COVERAGECONTROL_BIVARIATENORMALDISTRIBUTION_H_

#include <cmath>
#include <iostream>
#include "constants.h"
#include "parameters.h"
#include "typedefs.h"

namespace CoverageControl {
	class BivariateNormalDistribution {

		private:

			Point2 mean_; // Mean of the distribution
			Point2 sigma_; // variance is sigma^2
			double rho_; // Correlation coefficient
			double peak_val_ = 1; // Can be used to scale the distribution so that integration is peak_val_
			bool is_circular_ = false;

		public:

			// Default constructor for standard distribution
			BivariateNormalDistribution () {
				is_circular_ = true;
				sigma_ = Point2(1, 1);
				mean_ = Point2(0, 0);
				rho_ = 0;
			}

			// Constructor for circular distribution
			BivariateNormalDistribution (Point2 const mean, double const sigma, double const peak_val = 1) {
				is_circular_ = true;
				sigma_ = Point2(sigma, sigma);
				mean_ = mean;
				rho_ = 0;
				peak_val_ = peak_val;
			}

			// Constructor for general distribution
			BivariateNormalDistribution (Point2 const mean, Point2 const sigma, double const rho, double const peak_val = 1) {
				if(rho_ > 0) {
					is_circular_ = false;
				} else {
					is_circular_ = true;
				}
				sigma_ = sigma;
				mean_ = mean;
				rho_ = 0;
				peak_val_ = peak_val;
			}

			Point2 GetMean() const { return mean_; }
			Point2 GetSigma() const { return sigma_; }
			double GetRho() const { return rho_; }
			double GetScale() const { return peak_val_; }

			// Transform Point for standard distribution N~((0,0), 1)
			Point2 TransformPoint(Point2 const &in_point) const {
				if(is_circular_) {
					return Point2((in_point - mean_)/sigma_.x());
				} else {
					Point2 translated = in_point - mean_;
					Point2 normalized(translated.x()/sigma_.x(), translated.y()/sigma_.y());
					return Point2((normalized.x() - rho_ * normalized.y()) / (std::sqrt(1-rho_*rho_)), normalized.y());
				}
			}

			// Double integration from point.x() -> inf and point.y() -> inf
			double IntegrateQuarterPlane (Point2 const &point) const {
				auto transformed_point = TransformPoint(point);
				return peak_val_ * std::erfc(transformed_point.x() * kOneBySqrt2) * std::erfc(transformed_point.y() * kOneBySqrt2)/4.;
			}
	};

} /* namespace CoverageControl */
#endif /* _COVERAGECONTROL_BIVARIATENORMALDISTRIBUTION_H_ */
