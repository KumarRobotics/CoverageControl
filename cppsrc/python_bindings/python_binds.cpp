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
 * \file python_binds.cpp
 * \brief Python bindings for the CoverageControl library using pybind11 (\ref
 * core_binds.h)
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl_bind.h>

#include "core_binds.h"

PYBIND11_MODULE(_core, m) {
  CoverageControl::pyCoverageControl_core(m);
  CoverageControl::pyCoverageControl_core_coverage_system(m);
  CoverageControl::pyCoverageControl_core_cuda_utils(m);

#ifdef CoverageControl_VERSION
  m.attr("__version__") = CoverageControl_VERSION;
#else
  m.attr("__version__") = "dev";
#endif
}
