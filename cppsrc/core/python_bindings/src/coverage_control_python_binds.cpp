#include <pybind11/pybind11.h>
#include <pybind11/stl_bind.h>
#include <pybind11/eigen.h>

#include "../core_binds.h"

#include <CoverageControl/coverage_system.h>
using namespace CoverageControl;

PYBIND11_MODULE(pyCoverageControl, m) {
	pyCoverageControl_core(m);
	pyCoverageControl_core_coverage_system(m);
}

