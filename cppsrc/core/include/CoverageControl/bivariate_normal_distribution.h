/*!
 * This file is part of the CoverageControl library.
 * This file contains a class for Bivariate Normal Distribution.
 * The distribution is in it general form.
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

#ifndef COVERAGECONTROL_BIVARIATENORMALDISTRIBUTION_H_
#define COVERAGECONTROL_BIVARIATENORMALDISTRIBUTION_H_

#include <cmath>
#include <iostream>

#include "constants.h"
#include "parameters.h"
#include "typedefs.h"

namespace CoverageControl {

	/*!
	 * \brief Class for Bivariate Normal Distribution
	 *
	 * The class is used to represent a bivariate normal distribution.
	 * The distribution can be in its general form or a standard form.
	 * The standard form is \f$\mathcal N\sim ((0,0), 1)\f$ and the general form is \f$\mathcal N\sim ((\mu_x, \mu_y), (\sigma_x, \sigma_y), \rho)\f$.
	 * The class can be used to transform points from standard distribution to general distribution and vice versa.
	 * The class can also be used to integrate the distribution over a quarter plane.
	 */
	class BivariateNormalDistribution {
		private:

			Point2 mean_; /*!< Mean of the distribution */
			Point2 sigma_; /*!< Standard deviation of the distribution */
			double rho_ = 0; /*!< Correlation coefficient of the distribution */
			double scale_ = 1; /*!< Scale of the distribution */
			bool is_circular_ = false; /*!< Flag to check if the distribution is standard or general */

		public:

			/*!
			 * \brief Default constructor
			 *
			 * The default constructor initializes the distribution to standard form.
			 */
			BivariateNormalDistribution () {
				is_circular_ = true;
				sigma_ = Point2(1, 1);
				mean_ = Point2(0, 0);
				rho_ = 0;
			}

			/*!
			 * \brief Constructor for standard distribution
			 *
			 * The constructor initializes the distribution to standard form.
			 *
			 * @param mean Mean of the distribution
			 * @param sigma Standard deviation of the distribution
			 * @param scale Scale of the distribution
			 */
			BivariateNormalDistribution (Point2 const &mean, double const &sigma, double const scale = 1) {
				is_circular_ = true;
				sigma_ = Point2(sigma, sigma);
				mean_ = mean;
				rho_ = 0;
				scale_ = scale;
			}

			/*!
			 * \brief Constructor for general distribution
			 *
			 * The constructor initializes the distribution to general form.
			 *
			 * @param mean Mean of the distribution
			 * @param sigma Standard deviation of the distribution
			 * @param rho Correlation coefficient of the distribution
			 * @param scale Scale of the distribution
			 */
			BivariateNormalDistribution (Point2 const &mean, Point2 const &sigma, double const rho, double const scale = 1) {
				assert(rho_ < (1- kEps));
				if(rho_ > 0) {
					is_circular_ = false;
					rho_ = rho;
				} else {
					is_circular_ = true;
					rho_ = 0;
				}
				sigma_ = sigma;
				mean_ = mean;
				scale_ = scale;
			}

			//! Returns the mean of the distribution
			Point2 GetMean() const { return mean_; }
			//! Returns the standard deviation of the distribution
			Point2 GetSigma() const { return sigma_; }
			//! Returns the correlation coefficient of the distribution
			double GetRho() const { return rho_; }
			//! Returns the scale of the distribution
			double GetScale() const { return scale_; }

			/*!
			 * \brief Transforms a point from general distribution to standard distribution
			 *
			 * The function takes a point in general distribution and transforms it to standard distribution.
			 *
			 * @param in_point Point in general distribution
			 * @return Point in standard distribution
			 */
			Point2 TransformPoint(Point2 const &in_point) const {
				if(is_circular_) {
					return Point2((in_point - mean_)/sigma_.x());
				} else {
					Point2 translated = in_point - mean_;
					Point2 normalized(translated.x()/sigma_.x(), translated.y()/sigma_.y());
					return Point2((normalized.x() - rho_ * normalized.y()) / (std::sqrt(1-rho_*rho_)), normalized.y());
				}
			}

			/*!
			 * \brief Integrates the distribution over a quarter plane
			 * The function integrates the distribution over a quarter plane, i.e. \f$x > point.x()\f$ and \f$y > point.y()\f$.
			 *
			 * @param point Point in standard distribution
			 * @return Value of the integral
			 */
			double IntegrateQuarterPlane (Point2 const &point) const {
				auto transformed_point = TransformPoint(point);
				return scale_ * std::erfc(transformed_point.x() * kOneBySqrt2) * std::erfc(transformed_point.y() * kOneBySqrt2)/4.;
			}
	};

} /* namespace CoverageControl */
#endif /* COVERAGECONTROL_BIVARIATENORMALDISTRIBUTION_H_ */
