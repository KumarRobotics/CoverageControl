#include <torch/extension.h>
#include <pybind11/stl_bind.h>
#include <pybind11/eigen.h>

/* #include "../../coverage_control_python_binds.h" */

torch::Tensor d_sigmoid(torch::Tensor z) {
  auto s = torch::sigmoid(z);
  return (1 - s) * s;
}

void pyCoverageControl_torch(py::module &m);
PYBIND11_MODULE(pyCoverageControlTorch, m) {
	m.doc() = "CoverageControl library with torch extensions";
	/* pyCoverageControl_core(m); */
	pyCoverageControl_torch(m);
}

void pyCoverageControl_torch(py::module &m) {
  m.def("d_sigmoid", &d_sigmoid, "dSigmoid");
}
