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
  params.pNumGaussianFeatures = 2;
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
