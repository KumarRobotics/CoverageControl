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
 * \file core_binds.h
 * \brief Provides python bindings for the core CoverageControl library using
 * pybind11
 */

#ifndef CPPSRC_PYTHON_BINDINGS_CORE_BINDS_H_
#define CPPSRC_PYTHON_BINDINGS_CORE_BINDS_H_

#include <CoverageControl/Config.h>
#include <CoverageControl/bivariate_normal_distribution.h>
#include <CoverageControl/coverage_system.h>
#include <CoverageControl/parameters.h>
#include <CoverageControl/robot_model.h>
#include <CoverageControl/typedefs.h>
#include <CoverageControl/vec2d.h>
#include <CoverageControl/voronoi.h>
#include <CoverageControl/world_idf.h>

/* #include <CoverageControl/geographiclib_wrapper.h> */
#include <CoverageControl/algorithms/centralized_cvt.h>
#include <CoverageControl/algorithms/clairvoyant_cvt.h>
#include <CoverageControl/algorithms/decentralized_cvt.h>
#include <CoverageControl/algorithms/near_optimal_cvt.h>
#include <CoverageControl/algorithms/oracle_bang_explore_exploit.h>
#include <CoverageControl/algorithms/oracle_explore_exploit.h>
#include <CoverageControl/algorithms/simul_explore_exploit.h>

#include <vector>

namespace py = pybind11;
using namespace pybind11::literals;

namespace CoverageControl {
typedef std::vector<BivariateNormalDistribution> BNDVector;

void pyCoverageControl_core(py::module &m) {
  m.doc() = "CoverageControl library";

  m.def(
      "Point2", []() { return Point2(0, 0); },
      "Create a Point2 object with both values 0");
  m.def(
      "Point2", [](double const &a, double const &b) { return Point2(a, b); },
      "Create a Point2 object with values a and b");
  m.def(
      "Point2", [](Point2 const &p) { return Point2(p); },
      "Create a Point2 object with values from another Point2 object");

  m.def(
      "Point2f", []() { return Point2f(0, 0); },
      "Create a Point2f object with both values 0");
  m.def(
      "Point2f", [](float const &a, float const &b) { return Point2f(a, b); },
      "Create a Point2f object with values a and b");

  py::bind_vector<std::vector<double>>(m, "DblVector");
  py::bind_vector<std::vector<std::vector<double>>>(m, "DblVectorVector");
  py::bind_vector<std::vector<int>>(m, "intVector");

  py::bind_vector<PointVector>(m, "PointVector");
  py::bind_vector<std::vector<Point3>>(m, "Point3Vector");
  py::bind_vector<std::vector<MapType>>(m, "MapTypeVector");

  py::class_<PolygonFeature>(m, "PolygonFeature")
      .def(py::init<>())
      .def(py::init<PointVector const &, float const>())
      .def_readwrite("poly", &PolygonFeature::poly)
      .def_readwrite("imp", &PolygonFeature::imp)
      .def_readwrite("size", &PolygonFeature::size);
  py::bind_vector<std::vector<PolygonFeature>>(m, "PolygonVector");

  py::class_<VoronoiCell>(m, "VoronoiCell")
      .def_readonly("cell", &VoronoiCell::cell)
      .def_property_readonly("mass", &VoronoiCell::mass)
      .def_property_readonly("centroid", &VoronoiCell::centroid)
      .def_readonly("site", &VoronoiCell::site);
  py::bind_vector<std::vector<VoronoiCell>>(m, "VoronoiCells");

  py::class_<BivariateNormalDistribution>(m, "BivariateNormalDistribution")
      .def(py::init<>())
      .def(py::init<Point2 const, double const, double const>())
      .def(py::init<Point2 const, Point2 const, double const, double const>())
      .def("GetMean", &BivariateNormalDistribution::GetMean)
      .def("GetSigma", &BivariateNormalDistribution::GetSigma)
      .def("GetRho", &BivariateNormalDistribution::GetRho)
      .def("GetScale", &BivariateNormalDistribution::GetScale);

  py::bind_vector<BNDVector>(m, "BNDVector");

  py::class_<WorldIDF>(m, "WorldIDF")
      .def(py::init<Parameters const &>())
      .def(py::init<Parameters const &, std::string const &>())
      .def(py::init<Parameters const &, MapType const &>())
      .def("AddUniformDistributionPolygon",
           &WorldIDF::AddUniformDistributionPolygon)
      .def("AddNormalDistribution",
           py::overload_cast<BivariateNormalDistribution const &>(
               &WorldIDF::AddNormalDistribution))
      .def("AddNormalDistribution", py::overload_cast<BNDVector const &>(
                                        &WorldIDF::AddNormalDistribution))
      .def("GenerateMap", &WorldIDF::GenerateMap)
      .def("GenerateMapCPU", &WorldIDF::GenerateMapCPU)
#ifdef COVERAGECONTROL_WITH_CUDA
      .def("GenerateMapCuda", py::overload_cast<>(&WorldIDF::GenerateMapCuda))
#endif
      .def("GetWorldMap", &WorldIDF::GetWorldMap,
           py::return_value_policy::reference_internal)
      .def("GetWorldMapMutable", &WorldIDF::GetWorldMapMutable,
           py::return_value_policy::reference_internal)
      .def("WriteWorldMap", &WorldIDF::WriteWorldMap)
      .def("GetNormalizationFactor", &WorldIDF::GetNormalizationFactor)
      .def("PrintMapSize", &WorldIDF::PrintMapSize)
      .def("LoadMap", &WorldIDF::LoadMap)
      .def("WriteDistributions", &WorldIDF::WriteDistributions)
      .def("GetNumFeatures", &WorldIDF::GetNumFeatures);

  py::class_<RobotModel>(m, "RobotModel")
      .def(py::init<Parameters const &, Point2 const, WorldIDF const>())
      .def("StepControl", &RobotModel::StepControl)
      .def("SetRobotPosition", &RobotModel::SetRobotPosition)
      .def("SetGlobalRobotPosition", &RobotModel::SetGlobalRobotPosition)
      .def("GetGlobalStartPosition", &RobotModel::GetGlobalStartPosition)
      .def("GetGlobalCurrentPosition", &RobotModel::GetGlobalCurrentPosition)
      .def("GetRobotLocalMap", &RobotModel::GetRobotLocalMap,
           py::return_value_policy::reference_internal)
      .def("GetRobotSystemMap", &RobotModel::GetRobotSystemMap,
           py::return_value_policy::reference_internal)
      .def("GetSensorView", &RobotModel::GetSensorView,
           py::return_value_policy::reference_internal)
      .def("GetExplorationMap", &RobotModel::GetExplorationMap,
           py::return_value_policy::reference_internal)
      .def("GetObstacleMap", &RobotModel::GetObstacleMap,
           py::return_value_policy::reference_internal);

  py::class_<OracleExploreExploit>(m, "OracleExploreExploit")
      .def(py::init<Parameters const &, size_t const &, CoverageSystem &>())
      .def("ComputeActions", &OracleExploreExploit::ComputeActions)
      .def("IsConverged", &OracleExploreExploit::IsConverged)
      .def("GetActions", &OracleExploreExploit::GetActions)
      .def("SetGoals", &OracleExploreExploit::SetGoals)
      .def("GetGoals", &OracleExploreExploit::GetGoals)
      .def("GetVoronoiCells", &OracleExploreExploit::GetVoronoiCells,
           py::return_value_policy::copy)
      .def("GetOracleMap", &OracleExploreExploit::GetOracleMap,
           py::return_value_policy::reference_internal);

  py::class_<OracleSimulExploreExploit>(m, "OracleSimulExploreExploit")
      .def(py::init<Parameters const &, size_t const &, CoverageSystem &>())
      .def("ComputeActions", &OracleSimulExploreExploit::ComputeActions)
      .def("IsConverged", &OracleSimulExploreExploit::IsConverged)
      .def("GetActions", &OracleSimulExploreExploit::GetActions)
      .def("GetRobotStatus", &OracleSimulExploreExploit::GetRobotStatus)
      .def("SetGoals", &OracleSimulExploreExploit::SetGoals)
      .def("GetGoals", &OracleSimulExploreExploit::GetGoals);

  py::class_<NearOptimalCVT>(m, "NearOptimalCVT")
      .def(py::init<Parameters const &, CoverageSystem &>())
      .def(py::init<Parameters const &, size_t const &, CoverageSystem &>())
      .def("ComputeActions", &NearOptimalCVT::ComputeActions)
      .def("GetActions", &NearOptimalCVT::GetActions)
      .def("IsConverged", &NearOptimalCVT::IsConverged)
      .def("GetGoals", &NearOptimalCVT::GetGoals)
      .def("GetVoronoi", &NearOptimalCVT::GetVoronoi);

  py::class_<ClairvoyantCVT>(m, "ClairvoyantCVT")
      .def(py::init<Parameters const &, CoverageSystem &>())
      .def(py::init<Parameters const &, size_t const &, CoverageSystem &>())
      .def("ComputeActions", &ClairvoyantCVT::ComputeActions)
      .def("IsConverged", &ClairvoyantCVT::IsConverged)
      .def("GetActions", &ClairvoyantCVT::GetActions)
      .def("GetGoals", &ClairvoyantCVT::GetGoals)
      .def("GetVoronoi", &ClairvoyantCVT::GetVoronoi);

  py::class_<DecentralizedCVT>(m, "DecentralizedCVT")
      .def(py::init<Parameters const &, CoverageSystem &>())
      .def(py::init<Parameters const &, size_t const &, CoverageSystem &>())
      .def("ComputeActions", &DecentralizedCVT::ComputeActions)
      .def("IsConverged", &DecentralizedCVT::IsConverged)
      .def("GetActions", &DecentralizedCVT::GetActions)
      .def("GetGoals", &DecentralizedCVT::GetGoals);

  py::class_<CentralizedCVT>(m, "CentralizedCVT")
      .def(py::init<Parameters const &, CoverageSystem &>())
      .def(py::init<Parameters const &, size_t const &, CoverageSystem &>())
      .def("ComputeActions", &CentralizedCVT::ComputeActions)
      .def("IsConverged", &CentralizedCVT::IsConverged)
      .def("GetActions", &CentralizedCVT::GetActions)
      .def("GetGoals", &CentralizedCVT::GetGoals)
      .def("GetVoronoi", &CentralizedCVT::GetVoronoi);

  py::class_<OracleBangExploreExploit>(m, "OracleBangExploreExploit")
      .def(py::init<Parameters const &, size_t const &, CoverageSystem &>())
      .def("ComputeActions", &OracleBangExploreExploit::ComputeActions)
      .def("IsConverged", &OracleBangExploreExploit::IsConverged)
      .def("GetActions", &OracleBangExploreExploit::GetActions)
      .def("GetRobotStatus", &OracleBangExploreExploit::GetRobotStatus)
      .def("SetGoals", &OracleBangExploreExploit::SetGoals)
      .def("GetGoals", &OracleBangExploreExploit::GetGoals)
      .def("GetVoronoiCells", &OracleBangExploreExploit::GetVoronoiCells,
           py::return_value_policy::copy);

  py::class_<Parameters>(m, "Parameters")
      .def(py::init<>())
      .def(py::init<std::string const &>())
      .def("SetConfig", &Parameters::SetConfig)
      .def("PrintParameters", &Parameters::PrintParameters)
      .def_readwrite("pNumRobots", &Parameters::pNumRobots)
      .def_readwrite("pNumGaussianFeatures", &Parameters::pNumGaussianFeatures)
      .def_readwrite("pNumPolygons", &Parameters::pNumPolygons)
      .def_readwrite("pMaxVertices", &Parameters::pMaxVertices)
      .def_readwrite("pPolygonRadius", &Parameters::pPolygonRadius)
      .def_readwrite("pPlotScale", &Parameters::pPlotScale)
      .def_readwrite("pResolution", &Parameters::pResolution)
      .def_readwrite("pWorldMapSize", &Parameters::pWorldMapSize)
      .def_readwrite("pRobotMapSize", &Parameters::pRobotMapSize)
      .def_readwrite("pUnknownImportance", &Parameters::pUnknownImportance)
      .def_readwrite("pLocalMapSize", &Parameters::pLocalMapSize)
      .def_readwrite("pCommunicationRange", &Parameters::pCommunicationRange)
      .def_readwrite("pRobotInitDist", &Parameters::pRobotInitDist)
      .def_readwrite("pUpdateRobotMap", &Parameters::pUpdateRobotMap)
      .def_readwrite("pUpdateSensorView", &Parameters::pUpdateSensorView)
      .def_readwrite("pSensorSize", &Parameters::pSensorSize)
      .def_readwrite("pTimeStep", &Parameters::pTimeStep)
      .def_readwrite("pMaxRobotSpeed", &Parameters::pMaxRobotSpeed)
      .def_readwrite("pEpisodeSteps", &Parameters::pEpisodeSteps)
      .def_readwrite("pCheckOscillations", &Parameters::pCheckOscillations)
      .def_readwrite("pTruncationBND", &Parameters::pTruncationBND)
      .def_readwrite("pNorm", &Parameters::pNorm)
      .def_readwrite("pMinSigma", &Parameters::pMinSigma)
      .def_readwrite("pMaxSigma", &Parameters::pMaxSigma)
      .def_readwrite("pMinPeak", &Parameters::pMinPeak)
      .def_readwrite("pMaxPeak", &Parameters::pMaxPeak)
      .def_readwrite("pLloydNumTries", &Parameters::pLloydNumTries)
      .def_readwrite("pLloydMaxIterations", &Parameters::pLloydMaxIterations)
      .def_readwrite("pNumFrontiers", &Parameters::pNumFrontiers);

  /* py::class_<GeoLocalTransform>(m, "GeoLocalTransform") */
  /*     .def(py::init<double, double, double>()) */
  /*     .def("Reset", &GeoLocalTransform::Reset) */
  /*     .def("Forward", &GeoLocalTransform::Forward) */
  /*     .def("Reverse", &GeoLocalTransform::Reverse); */
}

void pyCoverageControl_core_coverage_system(py::module &m) {
  py::class_<CoverageSystem>(m, "CoverageSystem")
      .def(py::init<Parameters const &>())
      .def(py::init<Parameters const &, int const, int const>())
      .def(py::init<Parameters const &, int const, int const, int const>())
      .def(
          py::init<Parameters const &, WorldIDF const &, PointVector const &>())
      .def(
          py::init<Parameters const &, WorldIDF const &, std::string const &>())
      .def(py::init<Parameters const &, BNDVector const &,
                    PointVector const &>())
      .def("GetWorldMap", &CoverageSystem::GetWorldMap,
           py::return_value_policy::reference_internal)
      .def("GetWorldMapMutable", &CoverageSystem::GetWorldMapMutable,
           py::return_value_policy::reference_internal)
      .def("GetWorldIDFObject", &CoverageSystem::GetWorldIDFObject,
           py::return_value_policy::reference_internal)
      .def("GetWorldIDF", &CoverageSystem::GetWorldIDFObject,
           py::return_value_policy::reference_internal)
      .def("StepControl", &CoverageSystem::StepControl)
      .def("StepAction", &CoverageSystem::StepAction)
      .def("StepActions", &CoverageSystem::StepActions)
      .def("SetLocalRobotPositions", &CoverageSystem::SetLocalRobotPositions)
      .def("SetLocalRobotPosition", &CoverageSystem::SetLocalRobotPosition)
      .def("SetGlobalRobotPosition", &CoverageSystem::SetGlobalRobotPosition)
      .def("SetGlobalRobotPositions", &CoverageSystem::SetGlobalRobotPositions)
      .def("GetRelativePositonsNeighbors",
           &CoverageSystem::GetRelativePositonsNeighbors)
      .def("SetRobotPositions", &CoverageSystem::SetRobotPositions)
      /* .def("GetRobotPositions", &CoverageSystem::GetRobotPositions) */
      /* .def("GetRobotPosition", &CoverageSystem::GetRobotPosition) */
      .def("GetRobotPosition", &CoverageSystem::GetRobotPosition,
           "Get Position of Robot", py::arg("robot_id"),
           py::arg("force_no_noise") = false)
      .def("GetRobotPositions", &CoverageSystem::GetRobotPositions,
           "Get Positions of Robots", py::arg("force_no_noise") = false)
      .def("GetRobotLocalMap", &CoverageSystem::GetRobotLocalMap,
           py::return_value_policy::reference_internal)
      .def("GetRobotMap", &CoverageSystem::GetRobotMap,
           py::return_value_policy::reference_internal)
      .def("GetRobotMapMutable", &CoverageSystem::GetRobotMapMutable,
           py::return_value_policy::reference_internal)
      .def("GetRobotSensorView", &CoverageSystem::GetRobotSensorView,
           py::return_value_policy::reference_internal)
      .def("GetCommunicationMaps", &CoverageSystem::GetCommunicationMaps)
      .def("GetRobotsInCommunication",
           &CoverageSystem::GetRobotsInCommunication)
      .def("GetSystemExploredIDFMap", &CoverageSystem::GetSystemExploredIDFMap,
           py::return_value_policy::reference_internal)
      .def("GetSystemExploredIDFMapMutable",
           &CoverageSystem::GetSystemExploredIDFMapMutable,
           py::return_value_policy::reference_internal)
      .def("ComputeVoronoiCells", &CoverageSystem::ComputeVoronoiCells,
           py::return_value_policy::reference_internal)
      .def("GetVoronoiCells", &CoverageSystem::GetVoronoiCells,
           py::return_value_policy::copy)
      .def("GetVoronoiCell", &CoverageSystem::GetVoronoiCell,
           py::return_value_policy::copy)
      .def("GetRobotObstacleMap", &CoverageSystem::GetRobotObstacleMap,
           py::return_value_policy::copy)
      .def("GetLocalVoronoiFeatures",
           py::overload_cast<int>(&CoverageSystem::GetLocalVoronoiFeatures),
           py::return_value_policy::copy)
      .def("GetLocalVoronoiFeatures",
           py::overload_cast<>(&CoverageSystem::GetLocalVoronoiFeatures),
           py::return_value_policy::copy)
      .def("GetRobotVoronoiFeatures",
           py::overload_cast<>(&CoverageSystem::GetRobotVoronoiFeatures),
           py::return_value_policy::copy)
      .def("GetRobotExplorationFeatures",
           py::overload_cast<>(&CoverageSystem::GetRobotExplorationFeatures),
           py::return_value_policy::copy)
      .def("GetRobotExplorationMap", &CoverageSystem::GetRobotExplorationMap,
           py::return_value_policy::reference_internal)
      .def("GetSystemExplorationMap", &CoverageSystem::GetSystemExplorationMap,
           py::return_value_policy::reference_internal)
      .def("GetSystemMap", &CoverageSystem::GetSystemMap,
           py::return_value_policy::reference_internal)
      .def("GetObjectiveValue", &CoverageSystem::GetObjectiveValue)
      .def("PlotSystemMap", py::overload_cast<std::string const &>(
                                &CoverageSystem::PlotSystemMap, py::const_))
      .def("PlotSystemMap", py::overload_cast<std::string const &, int const &>(
                                &CoverageSystem::PlotSystemMap, py::const_))
      .def("PlotWorldMap", &CoverageSystem::PlotWorldMap)
      .def("PlotInitMap", py::overload_cast<std::string const &>(
                              &CoverageSystem::PlotInitMap, py::const_))
      .def("PlotInitMap",
           py::overload_cast<std::string const &, std::string const &>(
               &CoverageSystem::PlotInitMap, py::const_))
      .def("PlotWorldMapRobots", &CoverageSystem::PlotWorldMapRobots)
      .def("PlotMapVoronoi",
           py::overload_cast<std::string const &, int const &>(
               &CoverageSystem::PlotMapVoronoi))
      .def("PlotMapVoronoi",
           py::overload_cast<std::string const &, int const &, Voronoi const &,
                             PointVector const &>(
               &CoverageSystem::PlotMapVoronoi, py::const_))
      .def("PlotRobotLocalMap", &CoverageSystem::PlotRobotLocalMap)
      // .def("PlotRobotIDFMap", &CoverageSystem::PlotRobotIDFMap)
      .def("PlotRobotExplorationMap", &CoverageSystem::PlotRobotExplorationMap)
      .def("PlotRobotSensorView", &CoverageSystem::PlotRobotSensorView)
      .def("PlotRobotSystemMap", &CoverageSystem::PlotRobotSystemMap)
      .def("PlotRobotObstacleMap", &CoverageSystem::PlotRobotObstacleMap)
      .def("PlotRobotCommunicationMaps",
           &CoverageSystem::PlotRobotCommunicationMaps)
      .def("GetExplorationRatio", &CoverageSystem::GetExplorationRatio)
      .def("GetWeightedExplorationRatio",
           &CoverageSystem::GetWeightedExplorationRatio)
      .def("RecordPlotData",
           py::overload_cast<>(&CoverageSystem::RecordPlotData))
      .def("RecordPlotData", py::overload_cast<std::string const &>(
                                 &CoverageSystem::RecordPlotData))
      .def("RenderRecordedMap", &CoverageSystem::RenderRecordedMap)
      .def("WriteEnvironment", &CoverageSystem::WriteEnvironment)
      .def("GetNumRobots", &CoverageSystem::GetNumRobots)
      .def("ClearRobotMaps", &CoverageSystem::ClearRobotMaps)
      .def("ClearExploredIDF", &CoverageSystem::ClearExploredIDF);
}

// CudaUtils
void pyCoverageControl_core_cuda_utils(py::module &m) {
  py::class_<CudaUtils>(m, "CudaUtils")
      .def_static("UseCuda", &CudaUtils::UseCuda)
      .def_static("SetUseCuda", &CudaUtils::SetUseCuda)
      .def_static("IsCudaAvailable", &CudaUtils::IsCudaAvailable);
}

}  // namespace CoverageControl

#endif  // CPPSRC_PYTHON_BINDINGS_CORE_BINDS_H_
