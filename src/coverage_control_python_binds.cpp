#include <vector>

#include <CoverageControl/vec2d.h>
#include <CoverageControl/typedefs.h>
#include <CoverageControl/parameters.h>
#include <CoverageControl/bivariate_normal_distribution.h>
#include <CoverageControl/world_idf.h>
#include <CoverageControl/robot_model.h>
#include <CoverageControl/coverage_system.h>

#include <pybind11/pybind11.h>
#include <pybind11/stl_bind.h>
#include <pybind11/eigen.h>


using namespace CoverageControl;
typedef std::vector<BivariateNormalDistribution> BNDVector;
namespace py = pybind11;
using namespace pybind11::literals;


PYBIND11_MODULE(pyCoverageControl, m) {
	m.doc() = "CoverageControl library";

	py::bind_vector<std::vector<double>>(m, "DblVector");

	py::class_<Parameters>(m, "Parameters")
		.def(py::init<>())
		.def(py::init<std::string const &>())
		.def("SetConfig", &Parameters::SetConfig)
		;

	py::class_<Point2>(m, "Point2")
		.def(py::init<>())
		.def(py::init<double const, double const>())
		.def_property("x", &Point2::x, &Point2::SetX)
		.def_property("y", &Point2::y, &Point2::SetY)
		.def("__repr__",
				[](const Point2 &a) {
				return "<CoverageControl.Point2: (" + std::to_string(a.x()) + "," + std::to_string(a.y()) + ")>";
				});

	py::bind_vector<PointVector>(m, "PointVector");

	py::class_<BivariateNormalDistribution>(m, "BivariateNormalDistribution")
		.def(py::init<>())
		.def(py::init<Point2 const, double const, double const>())
		.def(py::init<Point2 const, Point2 const, double const, double const>())
		;

	py::bind_vector<BNDVector>(m, "BNDVector");

	py::class_<WorldIDF>(m, "WorldIDF")
		.def(py::init<Parameters const &>())
		.def("AddNormalDistribution", py::overload_cast<BivariateNormalDistribution const &>(&WorldIDF::AddNormalDistribution))
		.def("AddNormalDistribution", py::overload_cast<BNDVector const &>(&WorldIDF::AddNormalDistribution))
		.def("GenerateMap", &WorldIDF::GenerateMap)
		.def("GenerateMapCuda", &WorldIDF::GenerateMapCuda)
		.def("GetWorldMap", &WorldIDF::GetWorldMap, py::return_value_policy::reference_internal)
		.def("WriteWorldMap", &WorldIDF::WriteWorldMap)
		.def("GetMaxValue", &WorldIDF::GetMaxValue)
		.def("PrintMapSize", &WorldIDF::PrintMapSize)
		;

	py::class_<RobotModel>(m, "RobotModel")
		.def(py::init<Parameters const &, Point2 const, WorldIDF const>())
		.def("StepControl", &RobotModel::StepControl)
		.def("UpdateRobotPosition", &RobotModel::UpdateRobotPosition)
		.def("GetGlobalStartPosition", &RobotModel::GetGlobalStartPosition)
		.def("GetGlobalCurrentPosition", &RobotModel::GetGlobalCurrentPosition)
		.def("GetAllPositions", &RobotModel::GetAllPositions)
		.def("GetRobotMap", &RobotModel::GetRobotMap)
		.def("GetRobotLocalMap", &RobotModel::GetRobotLocalMap)
		.def("GetSensorView", &RobotModel::GetSensorView)
		;

	py::class_<CoverageSystem>(m, "CoverageSystem")
		.def(py::init<Parameters const &, int const, int const>())
		.def(py::init<Parameters const &, WorldIDF const &, PointVector const &>())
		.def(py::init<Parameters const &, BNDVector const &, PointVector const &>())
		.def("GetWorldIDF", &CoverageSystem::GetWorldIDF, py::return_value_policy::reference_internal)
		.def("StepControl", &CoverageSystem::StepControl)
		.def("UpdateRobotPositions", &CoverageSystem::UpdateRobotPositions)
		.def("GetRobotPositions", &CoverageSystem::GetRobotPositions)
		.def("GetRobotLocalMap", &CoverageSystem::GetRobotLocalMap, py::return_value_policy::reference_internal)
		.def("GetRobotSensorView", &CoverageSystem::GetRobotSensorView, py::return_value_policy::reference_internal)
		.def("GetCommunicationMap", &CoverageSystem::GetCommunicationMap, py::return_value_policy::reference_internal)
		;

}
