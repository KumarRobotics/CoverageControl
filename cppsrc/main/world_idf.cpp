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
 * \file world_idf.cpp
 * \brief Program to test generation of IDF
 *
 * ```bash
 * ./world_idf
 * ```
 *
 */

#include <CoverageControl/coverage_system.h>
#include <CoverageControl/parameters.h>
#include <CoverageControl/typedefs.h>
#include <CoverageControl/world_idf.h>

#include <iostream>
#include <memory>
#include <string>

using CoverageControl::CoverageSystem;
using CoverageControl::Parameters;
using CoverageControl::Point2;
using CoverageControl::PointVector;
using CoverageControl::WorldIDF;

int main(int argc, char** argv) {
  CoverageControl::CudaUtils::SetUseCuda(false);
  Parameters params;
  params.pNumRobots = 1;
  params.pNumFeatures = 2;
  params.pNumPolygons = 20;
  params.pWorldMapSize = 1024;
  params.pMaxVertices = 7;
  params.pPolygonRadius = 128;

  CoverageSystem env(params);
  env.WriteEnvironment("pos", "env");
  env.PlotInitMap("init_map");
  CoverageControl::MapType map = env.GetWorldMap();
  CoverageControl::CudaUtils::SetUseCuda(true);
  WorldIDF world_idf(params, "env");
  CoverageSystem env1(params, world_idf, "pos");
  env1.WriteEnvironment("pos1", "env1");
  env1.PlotInitMap("init_map1");

  return 0;
}
