#include <torch/extension.h>
#include <iomanip>
#include <torch/torch.h>
#include <torch/script.h>
#include <pybind11/stl_bind.h>
#include <pybind11/eigen.h>

#include "../../../core/python_bindings/core_binds.h"
/* #include "../../include/CoverageControlTorch/type_conversions.h" */
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> EigenMat;

/* typedef std::vector<CoverageControl::BivariateNormalDistribution> BNDVector; */
/* typedef CoverageControlTorch::CoverageSystem CS; */
torch::Tensor d_sigmoid(torch::Tensor z) {
	auto s = torch::sigmoid(z);
	return (1 - s) * s;
}

torch::Tensor EigenToTensor(EigenMat const &M) {
	EigenMat M_copy = M;
	std::vector<int64_t> dims = {M_copy.rows(), M_copy.cols()};
	torch::Tensor T = torch::from_blob(M_copy.data(), dims).clone();
	return T;
}

torch::Tensor DblToTensor(double const &data) {
	torch::Tensor T = torch::zeros(1);
	T[0] = (float)data;
	return T;
}

torch::Tensor PointToTensor(Eigen::Vector2d const &p) {
	torch::Tensor T = torch::zeros(2);
	T[0] = (float)p[0];
	T[1] = (float)p[1];
	return T;
}

torch::Tensor PointVecToTensor(std::vector<Eigen::Vector2d> const &vec) {
	int vector_size = (int)vec.size();
	std::vector<int64_t> dims = {vector_size, 2};
	torch::Tensor T = torch::zeros(dims);
	for (int i = 0; i < vec.size(); ++i) {
		T[i] = PointToTensor(vec[i]);
	}
	return T;
}


void pyCoverageControl_torch(py::module &m);
PYBIND11_MODULE(pyCoverageControlTorch, m) {
	m.doc() = "CoverageControl library with torch extensions";
	CoverageControl::pyCoverageControl_core(m);
	CoverageControl::pyCoverageControl_core_coverage_system(m);
	m.def("EigenToTensor", &EigenToTensor);
	m.def("DblToTensor", &DblToTensor);
	m.def("PointToTensor", &PointToTensor);
	m.def("PointVecToTensor", &PointVecToTensor);
	/* pyCoverageControl_torch(m); */
	/* m.def("ToTensor", py::overload_cast<EigenMat const &>(&CoverageControlTorch::ToTensor)); */
	/* m.def("ToTensor", py::overload_cast<Eigen::Vector2d const &>(&CoverageControlTorch::ToTensor)); */
	/* m.def("ToTensor", py::overload_cast<std::vector<double> const &>(&CoverageControlTorch::ToTensor)); */
	/* m.def("ToTensor", py::overload_cast<std::vector<std::vector<double>> const &>(&CoverageControlTorch::ToTensor)); */
	/* m.def("ToTensor", py::overload_cast<std::vector<Eigen::Vector2d> const &>(&CoverageControlTorch::ToTensor)); */
}

/* void pyCoverageControl_torch(py::module &m) { */
/* 	m.def("d_sigmoid", &d_sigmoid, "dSigmoid"); */
/* 	py::class_<CS>(m, "CoverageSystem") */
/* 		.def(py::init<Parameters const &, int const, int const>()) */
/* 		.def(py::init<Parameters const &, CoverageControl::WorldIDF const &, CoverageControl::PointVector const &>()) */
/* 		.def(py::init<Parameters const &, CoverageControl::WorldIDF const &, std::string const &>()) */
/* 		.def(py::init<Parameters const &, BNDVector const &, PointVector const &>()) */
/* 		.def("GetAllRobotsLocalMaps", &CS::GetAllRobotsLocalMaps, py::return_value_policy::reference_internal) */
/* 		.def("GetAllRobotsObstacleMaps", &CS::GetAllRobotsObstacleMaps, py::return_value_policy::reference_internal) */
/* 		.def("GetAllRobotsCommunicationMaps", &CS::GetAllRobotsCommunicationMaps, py::return_value_policy::reference_internal) */
/* 		.def("GetEdgeWeights", &CS::GetEdgeWeights, py::return_value_policy::reference_internal) */
/* 		.def("GetWorldIDF", &CS::GetWorldIDF, py::return_value_policy::reference_internal) */
/* 		.def("GetWorldIDFObject", &CS::GetWorldIDFObject, py::return_value_policy::reference_internal) */
/* 		.def("StepControl", &CS::StepControl) */
/* 		.def("StepAction", &CS::StepAction) */
/* 		.def("StepActions", &CS::StepActions) */
/* 		.def("SetRobotPositions", &CS::SetRobotPositions) */
/* 		.def("GetRobotPositions", &CS::GetRobotPositions) */
/* 		.def("GetRobotPosition", &CS::GetRobotPosition) */
/* 		.def("GetRobotLocalMap", &CS::GetRobotLocalMap, py::return_value_policy::reference_internal) */
/* 		.def("GetRobotSensorView", &CS::GetRobotSensorView, py::return_value_policy::reference_internal) */
/* 		.def("GetCommunicationMap", &CS::GetCommunicationMap, py::return_value_policy::reference_internal) */
/* 		.def("GetRobotsInCommunication", &CS::GetRobotsInCommunication) */
/* 		.def("ComputeVoronoiCells", &CS::ComputeVoronoiCells, py::return_value_policy::reference_internal) */
/* 		.def("GetVoronoiCells", &CS::GetVoronoiCells, py::return_value_policy::copy) */
/* 		.def("GetVoronoiCell", &CS::GetVoronoiCell, py::return_value_policy::copy) */
/* 		.def("GetRobotObstacleMap", &CS::GetRobotObstacleMap, py::return_value_policy::copy) */
/* 		.def("GetLocalVoronoiFeatures", py::overload_cast<int>(&CS::GetLocalVoronoiFeatures), py::return_value_policy::copy) */
/* 		.def("GetLocalVoronoiFeatures", py::overload_cast<>(&CS::GetLocalVoronoiFeatures), py::return_value_policy::copy) */
/* 		.def("GetRobotVoronoiFeatures", py::overload_cast<>(&CS::GetRobotVoronoiFeatures), py::return_value_policy::copy) */
/* 		.def("GetRobotExplorationFeatures", py::overload_cast<>(&CS::GetRobotExplorationFeatures), py::return_value_policy::copy) */
/* 		.def("GetRobotExplorationMap", &CS::GetRobotExplorationMap, py::return_value_policy::reference_internal) */
/* 		.def("GetSystemMap", &CS::GetSystemMap, py::return_value_policy::reference_internal) */
/* 		.def("GetObjectiveValue", &CS::GetObjectiveValue) */
/* 		.def("PlotSystemMap", py::overload_cast<std::string const &, int const &>(&CS::PlotSystemMap, py::const_)) */
/* 		.def("PlotWorldMap", &CS::PlotWorldMap) */
/* 		.def("PlotInitMap", &CS::PlotInitMap) */
/* 		.def("PlotWorldMapRobots", &CS::PlotWorldMapRobots) */
/* 		.def("PlotMapVoronoi", &CS::PlotMapVoronoi) */
/* 		.def("GetExplorationRatio", &CS::GetExplorationRatio) */
/* 		.def("GetWeightedExplorationRatio", &CS::GetWeightedExplorationRatio) */
/* 		.def("RecordPlotData", py::overload_cast<>(&CS::RecordPlotData)) */
/* 		.def("RenderRecordedMap", &CS::RenderRecordedMap) */
/* 		.def("WriteEnvironment", &CS::WriteEnvironment) */
/* 		.def("GetNumRobots", &CS::GetNumRobots) */
/* 		; */
/* } */
