#ifndef COVERAGECONTROL_PYTHON_BINDS_H_
#define COVERAGECONTROL_PYTHON_BINDS_H_

#include <vector>
#include <CoverageControl/vec2d.h>
#include <CoverageControl/typedefs.h>
#include <CoverageControl/parameters.h>
#include <CoverageControl/bivariate_normal_distribution.h>
#include <CoverageControl/world_idf.h>
#include <CoverageControl/robot_model.h>
#include <CoverageControl/coverage_system.h>
#include <CoverageControl/voronoi.h>
#include <CoverageControl/geographiclib_wrapper.h>
#include <CoverageControl/algorithms/oracle_global_offline.h>
#include <CoverageControl/algorithms/lloyd_local_voronoi.h>
#include <CoverageControl/algorithms/oracle_explore_exploit.h>
#include <CoverageControl/algorithms/oracle_bang_explore_exploit.h>
#include <CoverageControl/algorithms/simul_explore_exploit.h>
#include <CoverageControl/algorithms/lloyd_global_online.h>
#include <CoverageControl/algorithms/lloyd_local_sensor_global_comm.h>

#include <iostream>


namespace py = pybind11;
using namespace pybind11::literals;

namespace CoverageControl {
typedef std::vector<BivariateNormalDistribution> BNDVector;

	void pyCoverageControl_core(py::module &m) {
		m.doc() = "CoverageControl library";

		m.def("Point2", []() {return Point2(0, 0);});
		m.def("Point2", [](double const &a, double const &b) { return Point2(a, b);});


		py::bind_vector<std::vector<double>>(m, "DblVector");
		py::bind_vector<std::vector<std::vector<double>>>(m, "DblVectorVector");
		py::bind_vector<std::vector<int>>(m, "intVector");

		m.def("Point2", []() {return Point2(0, 0);});
		m.def("Point2", [](double const &a, double const &b) { return Point2(a, b);});

		py::bind_vector<PointVector>(m, "PointVector");
		py::bind_vector<std::vector<Point3>>(m, "Point3Vector");

		py::class_<PolygonFeature>(m, "PolygonFeature")
			.def(py::init<>())
			.def(py::init<PointVector const &, float const>())
			.def_readwrite("poly", &PolygonFeature::poly)
			.def_readwrite("imp", &PolygonFeature::imp)
			.def_readwrite("size", &PolygonFeature::size)
			;
		py::bind_vector<std::vector<PolygonFeature>>(m, "PolygonVector");

		py::class_<Edge>(m, "Edge")
			.def(py::init<double const, double const, double const, double const>())
			.def_readwrite("x1", &Edge::x1)
			.def_readwrite("y1", &Edge::y1)
			.def_readwrite("x2", &Edge::x2)
			.def_readwrite("y2", &Edge::y2)
			;
		py::bind_vector<std::vector<Edge>>(m, "EdgeList");

		py::class_<VoronoiCell>(m, "VoronoiCell")
			.def_readonly("cell", &VoronoiCell::cell)
			.def_property_readonly("mass", &VoronoiCell::mass)
			.def_property_readonly("centroid", &VoronoiCell::centroid)
			.def_readonly("site", &VoronoiCell::site)
			;
		py::bind_vector<std::vector<VoronoiCell>>(m, "VoronoiCells");

		py::class_<BivariateNormalDistribution>(m, "BivariateNormalDistribution")
			.def(py::init<>())
			.def(py::init<Point2 const, double const, double const>())
			.def(py::init<Point2 const, Point2 const, double const, double const>())
			;

		py::bind_vector<BNDVector>(m, "BNDVector");

		py::class_<WorldIDF>(m, "WorldIDF")
			.def(py::init<Parameters const &>())
			.def(py::init<Parameters const &, std::string const &>())
			.def("AddUniformDistributionPolygon", &WorldIDF::AddUniformDistributionPolygon)
			.def("AddNormalDistribution", py::overload_cast<BivariateNormalDistribution const &>(&WorldIDF::AddNormalDistribution))
			.def("AddNormalDistribution", py::overload_cast<BNDVector const &>(&WorldIDF::AddNormalDistribution))
			.def("GenerateMap", &WorldIDF::GenerateMap)
			.def("GenerateMapCuda", py::overload_cast<>(&WorldIDF::GenerateMapCuda))
			.def("GenerateMapCuda", py::overload_cast<float const, float const, int const>(&WorldIDF::GenerateMapCuda))
			.def("GetWorldMap", &WorldIDF::GetWorldMap, py::return_value_policy::reference_internal)
			.def("WriteWorldMap", &WorldIDF::WriteWorldMap)
			.def("GetNormalizationFactor", &WorldIDF::GetNormalizationFactor)
			.def("PrintMapSize", &WorldIDF::PrintMapSize)
			.def("LoadMap", &WorldIDF::LoadMap)
			;

		py::class_<RobotModel>(m, "RobotModel")
			.def(py::init<Parameters const &, Point2 const, WorldIDF const>())
			.def("StepControl", &RobotModel::StepControl)
			.def("SetRobotPosition", &RobotModel::SetRobotPosition)
			.def("GetGlobalStartPosition", &RobotModel::GetGlobalStartPosition)
			.def("GetGlobalCurrentPosition", &RobotModel::GetGlobalCurrentPosition)
			.def("GetRobotMap", &RobotModel::GetRobotMap, py::return_value_policy::reference_internal)
			.def("GetRobotLocalMap", &RobotModel::GetRobotLocalMap, py::return_value_policy::reference_internal)
			.def("GetSensorView", &RobotModel::GetSensorView, py::return_value_policy::reference_internal)
			.def("GetExplorationMap", &RobotModel::GetExplorationMap, py::return_value_policy::reference_internal)
			;

		py::class_<OracleExploreExploit>(m, "OracleExploreExploit")
			.def(py::init<Parameters const &, size_t const &, CoverageSystem &>())
			.def("Step", &OracleExploreExploit::Step)
			.def("GetActions", &OracleExploreExploit::GetActions)
			.def("SetGoals", &OracleExploreExploit::SetGoals)
			.def("GetGoals", &OracleExploreExploit::GetGoals)
			.def("GetVoronoiCells", &OracleExploreExploit::GetVoronoiCells, py::return_value_policy::copy)
			.def("GetOracleMap", &OracleExploreExploit::GetOracleMap, py::return_value_policy::reference_internal)
			;

		py::class_<OracleSimulExploreExploit>(m, "OracleSimulExploreExploit")
			.def(py::init<Parameters const &, size_t const &, CoverageSystem &>())
			.def("Step", &OracleSimulExploreExploit::Step)
			.def("GetActions", &OracleSimulExploreExploit::GetActions)
			.def("GetRobotStatus", &OracleSimulExploreExploit::GetRobotStatus)
			.def("SetGoals", &OracleSimulExploreExploit::SetGoals)
			.def("GetGoals", &OracleSimulExploreExploit::GetGoals)
			;

		py::class_<OracleGlobalOffline>(m, "OracleGlobalOffline")
			.def(py::init<Parameters const &, size_t const &, CoverageSystem &>())
			.def("Step", &OracleGlobalOffline::Step)
			.def("GetActions", &OracleGlobalOffline::GetActions)
			.def("GetGoals", &OracleGlobalOffline::GetGoals)
			.def("GetVoronoi", &OracleGlobalOffline::GetVoronoi)
			;

		py::class_<LloydGlobalOnline>(m, "LloydGlobalOnline")
			.def(py::init<Parameters const &, size_t const &, CoverageSystem &>())
			.def("Step", &LloydGlobalOnline::Step)
			.def("GetActions", &LloydGlobalOnline::GetActions)
			.def("GetGoals", &LloydGlobalOnline::GetGoals)
			.def("GetVoronoi", &LloydGlobalOnline::GetVoronoi)
			;

		py::class_<LloydLocalVoronoi>(m, "LloydLocalVoronoi")
			.def(py::init<Parameters const &, size_t const &, CoverageSystem &>())
			.def("Step", &LloydLocalVoronoi::Step)
			.def("GetActions", &LloydLocalVoronoi::GetActions)
			.def("GetGoals", &LloydLocalVoronoi::GetGoals)
			;

		py::class_<LloydLocalSensorGlobalComm>(m, "LloydLocalSensorGlobalComm")
			.def(py::init<Parameters const &, size_t const &, CoverageSystem &>())
			.def("Step", &LloydLocalSensorGlobalComm::Step)
			.def("GetActions", &LloydLocalSensorGlobalComm::GetActions)
			.def("GetGoals", &LloydLocalSensorGlobalComm::GetGoals)
			.def("GetVoronoi", &LloydLocalSensorGlobalComm::GetVoronoi)
			;

		py::class_<OracleBangExploreExploit>(m, "OracleBangExploreExploit")
			.def(py::init<Parameters const &, size_t const &, CoverageSystem &>())
			.def("Step", &OracleBangExploreExploit::Step)
			.def("GetActions", &OracleBangExploreExploit::GetActions)
			.def("GetRobotStatus", &OracleBangExploreExploit::GetRobotStatus)
			.def("SetGoals", &OracleBangExploreExploit::SetGoals)
			.def("GetGoals", &OracleBangExploreExploit::GetGoals)
			.def("GetVoronoiCells", &OracleBangExploreExploit::GetVoronoiCells, py::return_value_policy::copy)
			;


		py::class_<Parameters>(m, "Parameters")
			.def(py::init<>())
			.def(py::init<std::string const &>())
			.def("SetConfig", &Parameters::SetConfig)
			.def_readwrite("pNumRobots", &Parameters::pNumRobots)
			.def_readwrite("pNumFeatures", &Parameters::pNumFeatures)
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
			.def_readwrite("pTruncationBND", &Parameters::pTruncationBND)
			.def_readwrite("pNorm", &Parameters::pNorm)
			.def_readwrite("pMinSigma", &Parameters::pMinSigma)
			.def_readwrite("pMaxSigma", &Parameters::pMaxSigma)
			.def_readwrite("pMinPeak", &Parameters::pMinPeak)
			.def_readwrite("pMaxPeak", &Parameters::pMaxPeak)
			.def_readwrite("pLloydNumTries", &Parameters::pLloydNumTries)
			.def_readwrite("pLloydMaxIterations", &Parameters::pLloydMaxIterations)
			.def_readwrite("pNumFrontiers", &Parameters::pNumFrontiers)
			;

		py::class_<GeoLocalTransform>(m, "GeoLocalTransform")
			.def(py::init<double, double, double>())
			.def("Reset", &GeoLocalTransform::Reset)
			.def("Forward", &GeoLocalTransform::Forward)
			.def("Reverse", &GeoLocalTransform::Reverse)
			;
	}

	void pyCoverageControl_core_coverage_system(py::module &m) {
		py::class_<CoverageSystem>(m, "CoverageSystem")
			.def(py::init<Parameters const &, int const, int const>())
			.def(py::init<Parameters const &, WorldIDF const &, PointVector const &>())
			.def(py::init<Parameters const &, WorldIDF const &, std::string const &>())
			.def(py::init<Parameters const &, BNDVector const &, PointVector const &>())
			.def("GetWorldIDF", &CoverageSystem::GetWorldIDF, py::return_value_policy::reference_internal)
			.def("GetWorldIDFObject", &CoverageSystem::GetWorldIDFObject, py::return_value_policy::reference_internal)
			.def("StepControl", &CoverageSystem::StepControl)
			.def("StepAction", &CoverageSystem::StepAction)
			.def("StepActions", &CoverageSystem::StepActions)
			.def("SetLocalRobotPositions", &CoverageSystem::SetLocalRobotPositions)
			.def("SetLocalRobotPosition", &CoverageSystem::SetLocalRobotPosition)
			.def("GetRelativePositonsNeighbors", &CoverageSystem::GetRelativePositonsNeighbors)
			.def("SetRobotPositions", &CoverageSystem::SetRobotPositions)
			.def("GetRobotPositions", &CoverageSystem::GetRobotPositions)
			.def("GetRobotPosition", &CoverageSystem::GetRobotPosition)
			.def("GetRobotLocalMap", &CoverageSystem::GetRobotLocalMap, py::return_value_policy::reference_internal)
			.def("GetRobotSensorView", &CoverageSystem::GetRobotSensorView, py::return_value_policy::reference_internal)
			.def("GetCommunicationMap", &CoverageSystem::GetCommunicationMap, py::return_value_policy::reference_internal)
			.def("GetRobotsInCommunication", &CoverageSystem::GetRobotsInCommunication)
			.def("ComputeVoronoiCells", &CoverageSystem::ComputeVoronoiCells, py::return_value_policy::reference_internal)
			.def("GetVoronoiCells", &CoverageSystem::GetVoronoiCells, py::return_value_policy::copy)
			.def("GetVoronoiCell", &CoverageSystem::GetVoronoiCell, py::return_value_policy::copy)
			.def("GetRobotObstacleMap", &CoverageSystem::GetRobotObstacleMap, py::return_value_policy::copy)
			.def("GetLocalVoronoiFeatures", py::overload_cast<int>(&CoverageSystem::GetLocalVoronoiFeatures), py::return_value_policy::copy)
			.def("GetLocalVoronoiFeatures", py::overload_cast<>(&CoverageSystem::GetLocalVoronoiFeatures), py::return_value_policy::copy)
			.def("GetRobotVoronoiFeatures", py::overload_cast<>(&CoverageSystem::GetRobotVoronoiFeatures), py::return_value_policy::copy)
			.def("GetRobotExplorationFeatures", py::overload_cast<>(&CoverageSystem::GetRobotExplorationFeatures), py::return_value_policy::copy)
			.def("GetRobotExplorationMap", &CoverageSystem::GetRobotExplorationMap, py::return_value_policy::reference_internal)
			.def("GetSystemMap", &CoverageSystem::GetSystemMap, py::return_value_policy::reference_internal)
			.def("GetObjectiveValue", &CoverageSystem::GetObjectiveValue)
			.def("PlotSystemMap", py::overload_cast<std::string const &, int const &>(&CoverageSystem::PlotSystemMap, py::const_))
			.def("PlotWorldMap", &CoverageSystem::PlotWorldMap)
			.def("PlotInitMap", &CoverageSystem::PlotInitMap)
			.def("PlotWorldMapRobots", &CoverageSystem::PlotWorldMapRobots)
			.def("PlotMapVoronoi", py::overload_cast<std::string const &, int const &>(&CoverageSystem::PlotMapVoronoi))
			.def("PlotMapVoronoi", py::overload_cast<std::string const &, int const &, Voronoi const &, PointVector const &>(&CoverageSystem::PlotMapVoronoi, py::const_))
			.def("GetExplorationRatio", &CoverageSystem::GetExplorationRatio)
			.def("GetWeightedExplorationRatio", &CoverageSystem::GetWeightedExplorationRatio)
			.def("RecordPlotData", py::overload_cast<>(&CoverageSystem::RecordPlotData))
			.def("RecordPlotData", py::overload_cast<std::string const &>(&CoverageSystem::RecordPlotData))
			.def("RenderRecordedMap", &CoverageSystem::RenderRecordedMap)
			.def("WriteEnvironment", &CoverageSystem::WriteEnvironment)
			.def("GetNumRobots", &CoverageSystem::GetNumRobots)
			;
	}

}



#endif
