/*
 * This file is part of the CoverageControl library
 *
 * Author: Saurav Agarwal
 * Contact: sauravag@seas.upenn.edu, agr.saurav1@gmail.com
 * Repository: https://github.com/KumarRobotics/CoverageControl
 *
 * The CoverageControl library is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
 *
 * The CoverageControl library is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with CoverageControl library. If not, see <https://www.gnu.org/licenses/>.
 */

#include <torch/script.h> // One-stop header.
/* #include <torch/torch.h> */
#include <iostream>
#include <CoverageControlTorch/coverage_system.h>
#include <CoverageControlTorch/type_conversions.h>

using namespace CoverageControl;

int main(int argc, const char* argv[]) {

	Parameters params;
	std::vector<Point2> robot_positions;
	robot_positions.push_back(Point2(512, 512));
	robot_positions.push_back(robot_positions[0] - Point2(-60, 50));
	robot_positions.push_back(robot_positions[0] + Point2(-50, 60));
	robot_positions.push_back(robot_positions[0] - Point2(50, 50));
	robot_positions.push_back(robot_positions[0] + Point2(60, 60));
	robot_positions.push_back(robot_positions[0] - Point2(0, 50));
	robot_positions.push_back(robot_positions[0] + Point2(0, 60));
	robot_positions.push_back(robot_positions[0] - Point2(50, 0));
	robot_positions.push_back(robot_positions[0] + Point2(60, 0));

	robot_positions.push_back(robot_positions[0] + Point2(500, 0));

	WorldIDF world_idf(params);
	world_idf.AddNormalDistribution(BivariateNormalDistribution(Point2(500, 500), 10));
	world_idf.AddNormalDistribution(BivariateNormalDistribution(Point2(300, 500), 10));

	CoverageControlTorch::CoverageSystem env(params, world_idf, robot_positions);

	std::string base_dir = "./k3_params/";
	std::string py_dir = base_dir + "py/";
	std::string cpp_dir = base_dir + "cpp/";

	// Get names of all files in py_dir
	for (const auto & entry : std::filesystem::directory_iterator(py_dir)) {
		std::string filename = entry.path().filename();
		std::string py_filename = py_dir + filename;
		std::string cpp_filename = cpp_dir + filename;
		std::cout << filename << std::endl;
		torch::Tensor tensor = CoverageControlTorch::GetTensorFromBytes(py_filename);
		std::cout << filename << " " << tensor.sizes() << std::endl;
		torch::save(tensor, cpp_filename);
		torch::Tensor temp;
		torch::load(temp, cpp_filename);
		std::cout << temp.sizes() << std::endl;
		std::cout << torch::equal(tensor, temp) << std::endl;
	}
	/* int nlayers = 5; */
	/* int K = 3; */

	/* for(int l = 0; l < nlayers; ++l) { */
	/* 	std::cout << "Layer: " << l << std::endl; */
	/* 	std::string bname = "bias_" + std::to_string(l); */
	/* 	std::string bfilename = "./k3_params/py/" + bname + ".pt"; */
	/* 	torch::Tensor btensor = GetTensorFromBytes(bfilename); */
	/* 	std::cout << bname << " " << btensor.sizes() << std::endl; */
	/* 	torch::save(btensor, "./k3_params/cpp/" + bname + ".pt"); */
	/* 	for(int k = 0; k < K + 1; ++k) { */
	/* 		std::cout << "K: " << k << std::endl; */
	/* 		std::string name = "lin_" + std::to_string(l) + "_" + std::to_string(k); */
	/* 		std::string filename = "./k3_params/py/" + name + ".pt"; */
	/* 		torch::Tensor tensor = GetTensorFromBytes(filename); */
	/* 		std::cout << name << " " << tensor.sizes() << std::endl; */
	/* 		torch::save(tensor, "./k3_params/cpp/" + name + ".pt"); */
	/* 		torch::Tensor temp; */
	/* 		torch::load(temp, "./k3_params/cpp/" + name + ".pt"); */
	/* 		std::cout << temp.sizes() << std::endl; */
	/* 		std::cout << torch::equal(tensor, temp) << std::endl; */
	/* 	} */
	/* } */



	/* env.LoadCNNBackBoneJIT(argv[1]); */
	/* env.LoadCNNBackBone(argv[1]); */

	/* std::string tensor_file_name = std::string(argv[2]); */
	/* std::vector<char> f = get_the_bytes(tensor_file_name); */
	/* torch::IValue x = torch::pickle_load(f); */
	/* torch::Tensor my_tensor = x.toTensor(); */
	/* std::cout << my_tensor.sizes() << std::endl; */
	/* std::cout << torch::sum(my_tensor) << std::endl; */

	/* my_tensor = my_tensor.view({-1, my_tensor.size(-3), my_tensor.size(-2), my_tensor.size(-1)}); */
	/* torch::Tensor out = env.GetGNNFeatures(my_tensor); */
	/* torch::save(out, "out.pt"); */
	/* torch::jit::script::Module container = torch::jit::load(tensor_file_name); */
	/* torch::Tensor tensor = container.attr("tensor").toTensor(); */	
	/* std::cout << tensor.sizes() << std::endl; */
	/* std::cout << torch::sum(tensor) << std::endl; */



	/* env.PlotInitMap("./", "init"); */

	/* auto comm_map = env.GetAllRobotsCommunicationMaps(32); */
	/* torch::save(comm_map, "comm_map.pt"); */

	/* MapType world_map = env.GetWorldIDF(); */
	/* torch::Tensor world_map_tensor = CoverageControlTorch::ToTensor(world_map).clone(); */
	/* torch::save(world_map_tensor, "world_map.pt"); */

	/* torch::jit::script::Module torchvision_resizer_; */
	/* torchvision_resizer_ = torch::jit::load("./python/ts_jit/TorchVisionResize_32.pt"); */

	/* torch::Tensor output = torchvision_resizer_.forward({comm_map}).toTensor(); */
	/* torch::save(output, "comm_map_resized.pt"); */

	/* torch::jit::script::Module module; */
	/* try { */
	/* 	// Deserialize the ScriptModule from a file using torch::jit::load(). */
	/* 	module = torch::jit::load(argv[1]); */
	/* 	for (auto const &params : module.named_parameters()) { */
	/* 		std::cout << params.name << " " << params.value.sizes() << std::endl; */
	/* 		std::cout << params.value << std::endl; */
	/* 	} */
	/* } */
	/* catch (const c10::Error& e) { */
	/* 	std::cerr << "error loading the model\n"; */
	/* 	return -1; */
	/* } */

	std::cout << "ok\n";
	return 0;
}
