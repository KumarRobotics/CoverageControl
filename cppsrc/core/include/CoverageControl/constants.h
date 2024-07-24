/*
 * This file is part of the CoverageControl library
 *
 * Author: Saurav Agarwal
 * Contact: sauravag@seas.upenn.edu, agr.saurav1@gmail.com
 * Repository: https://github.com/KumarRobotics/CoverageControl
 *
 * Copyright (c) 2024, Saurav Agarwal
 *
 * The CoverageControl library is free software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * The CoverageControl library is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * CoverageControl library. If not, see <https://www.gnu.org/licenses/>.
 */

/*!
 * \file constants.h
 * \brief Constants for the CoverageControl library
 */

#ifndef CPPSRC_CORE_INCLUDE_COVERAGECONTROL_CONSTANTS_H_
#define CPPSRC_CORE_INCLUDE_COVERAGECONTROL_CONSTANTS_H_

#include <cmath>
#include <limits>

#include "CoverageControl/Config.h"

/*!
 * \namespace CoverageControl
 * \brief Namespace for the CoverageControl library
 */
namespace CoverageControl {
/*!
 * \addtogroup cpp_api_constants Constants
 * \ingroup cpp_api
 * @{
 */

double const kEps = 1e-10;          /*!< Epsilon for double comparison */
float const kEpsf = 1e-6f;          /*!< Epsilon for float comparison */
double const kLargeEps = 1e-4;      /*!< Large epsilon for double comparison */
double const kSqrt2 = std::sqrt(2); /*!< Square root of 2 */
double const kOneBySqrt2 = 1. / std::sqrt(2); /*!< 1 by square root of 2 */
float const kOneBySqrt2f = 1.f / sqrtf(2.f);  /*!< 1 by square root of 2 */
double const kInfD =
    std::numeric_limits<double>::infinity(); /*!< Infinity for double */
constexpr auto kMaxPrecision{std::numeric_limits<long double>::digits10 +
                             1}; /*!< Maximum precision for long double */

/*!
 * @}
 */

} /* namespace CoverageControl */
#endif  // CPPSRC_CORE_INCLUDE_COVERAGECONTROL_CONSTANTS_H_
