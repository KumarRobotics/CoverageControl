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
 * \file abstract_controller.h
 * \brief Contains the abstract class for coverage control algorithms
 */

#ifndef CPPSRC_CORE_INCLUDE_COVERAGECONTROL_ALGORITHMS_ABSTRACT_CONTROLLER_H_
#define CPPSRC_CORE_INCLUDE_COVERAGECONTROL_ALGORITHMS_ABSTRACT_CONTROLLER_H_

#include "CoverageControl/typedefs.h"

namespace CoverageControl {

/*!
 * \addtogroup cpp_api
 * @{
 * \class AbstractController
 * @}
 * The class AbstractController is an abstract class for coverage control
 *algorithms. It provides a common interface for all coverage control
 *algorithms. Pure virtual functions: GetActions and ComputeActions
 **/
class AbstractController {
 public:
  /*!
   * Pure virtual function to get the actions for the robots
   * \return The actions for the robots
   **/
  virtual PointVector GetActions() = 0;

  /*!
   * Pure virtual function to compute the actions for the robots
   * \return 0 if the actions are computed successfully, 1 otherwise
   **/
  virtual int ComputeActions() = 0;
};

} /* namespace CoverageControl */
#endif  // CPPSRC_CORE_INCLUDE_COVERAGECONTROL_ALGORITHMS_ABSTRACT_CONTROLLER_H_
