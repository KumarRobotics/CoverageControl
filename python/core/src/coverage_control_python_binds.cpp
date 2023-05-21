#include <pybind11/pybind11.h>
#include <pybind11/stl_bind.h>
#include <pybind11/eigen.h>

#include "../../coverage_control_python_binds.h"

PYBIND11_MODULE(pyCoverageControl, m) {
	pyCoverageControl_core(m);
}
