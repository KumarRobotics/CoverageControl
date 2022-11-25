#include <CoverageControl/vec2d.h>
#include <CoverageControl/typedefs.h>
#include <CoverageControl/bivariate_normal_distribution.h>
#include <CoverageControl/world_idf.h>
#include <CoverageControl/robot_model.h>
/* #include <CoverageControl/coverage_system.h> */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

using namespace CoverageControl;
namespace py = pybind11;
using namespace pybind11::literals;
PYBIND11_MODULE(pyCoverageControl, m) {
	m.doc() = "CoverageControl library";
	py::class_<Point2>(m, "Point2")
		.def(py::init<double const, double const>())
		.def_property("x", &Point2::x, &Point2::SetX)
		.def_property("y", &Point2::y, &Point2::SetY)
    .def("__repr__",
        [](const Point2 &a) {
            return "<CoverageControl.Point2: (" + std::to_string(a.x()) + "," + std::to_string(a.y()) + ")>";
        });

	py::class_<BivariateNormalDistribution>(m, "BivariateNormalDistribution")
		.def(py::init<>())
		.def(py::init<Point2 const, double const, double const>())
		.def(py::init<Point2 const, Point2 const, double const, double const>())
		;

	py::class_<WorldIDF>(m, "WorldIDF")
		.def(py::init<>())
		.def("AddNormalDistribution", &WorldIDF::AddNormalDistribution)
		.def("GenerateMap", &WorldIDF::GenerateMap)
		.def("GenerateMapCuda", &WorldIDF::GenerateMapCuda)
		.def("GetWorldMap", &WorldIDF::GetWorldMap, py::return_value_policy::reference_internal)
		.def("WriteWorldMap", &WorldIDF::WriteWorldMap)
		.def("GetMaxValue", &WorldIDF::GetMaxValue)
		.def("PrintMapSize", &WorldIDF::PrintMapSize)
		;

	py::class_<RobotModel>(m, "RobotModel")
		.def(py::init<Point2 const, WorldIDF const>())
		.def("StepControl", &RobotModel::StepControl)
		.def("UpdateRobotPosition", &RobotModel::UpdateRobotPosition)
		.def("GetGlobalStartPosition", &RobotModel::GetGlobalStartPosition)
		.def("GetGlobalCurrentPosition", &RobotModel::GetGlobalCurrentPosition)
		.def("GetAllPositions", &RobotModel::GetAllPositions)
		.def("GetRobotMap", &RobotModel::GetRobotMap)
		.def("GetRobotLocalMap", &RobotModel::GetRobotLocalMap)
		.def("GetSensorView", &RobotModel::GetSensorView)
		;

}
