#include <CoverageControl/vec2d.h>
#include <CoverageControl/typedefs.h>
/* #include <CoverageControl/coverage_system.h> */

#include <pybind11/pybind11.h>

using namespace CoverageControl;
namespace py = pybind11;
PYBIND11_MODULE(pyCoverageControl, m) {
	py::class_<Point2>(m, "Point2")
		.def(py::init<const double, const double>())
		.def_property("x", &Point2::x, &Point2::SetX)
		.def_property("y", &Point2::y, &Point2::SetY);
}
