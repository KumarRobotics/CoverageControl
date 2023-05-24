#include <torch/script.h>
#include <torch/torch.h>
#include <torchvision/vision.h>
#include <iostream>

#include <iomanip>
#include <iostream>
#include <random>
#include <filesystem>

#include <CoverageControlTorch/coverage_system.h>
#include <CoverageControlTorch/generate_dataset.h>

using CoverageControl::Point2;
using CoverageControl::BivariateNormalDistribution;
using CoverageControl::WorldIDF;
using namespace torch::indexing;

namespace CC = CoverageControl;
namespace CCT = CoverageControlTorch;

int main(int argc, char** argv) {
	/* if (argc != 2) { */
	/* 	std::cout << "Usage: ./data_generation <path_to_resizer_model>" << std::endl; */
	/* 	std::cout << "TorchVision JIT Transformer is required\n"; */
	/* 	return 1; */
	/* } */

	/* // Read first argument as path to resizer model */
	/* std::string resizer_model_path = argv[1]; */
	/* if (!std::filesystem::exists(resizer_model_path)) { */
	/* 	std::cout << "Error: " << resizer_model_path << " does not exist\n"; */
	/* 	return 1; */
	/* } */

	/* Parameters params; */

	/* std::vector <Point2> robot_positions; */
	/* robot_positions.push_back(Point2(100, 100)); */
	/* robot_positions.push_back(Point2(100, 10)); */
	/* robot_positions.push_back(Point2(10, 70)); */

	/* WorldIDF world_idf(params); */
	/* params.pNumRobots = robot_positions.size(); */
	/* world_idf.AddNormalDistribution(BivariateNormalDistribution(Point2(95, 95), 10)); */

	/* CCT::CoverageSystem env (params, world_idf, robot_positions); */
	/* torch::Tensor local_maps = torch::empty({2, params.pNumRobots, params.pLocalMapSize, params.pLocalMapSize}); */
	/* std::cout << std::setprecision(10) << std::endl; */
	/* env.GetAllRobotsLocalMaps(local_maps[0]); */
	/* env.GetAllRobotsLocalMaps(local_maps[1]); */
	/* print(local_maps[0].sum()); */
	/* std::cout << local_maps.sum() << std::endl; */
	/* std::cout << "shape: " << local_maps.sizes(); */
	/* torch::save(local_maps[0], "test.pt"); */

  /* torch::jit::script::Module resizer_model; */
	/* resizer_model = torch::jit::load(resizer_model_path); */

	/* std::cout << "shape of local_maps: " << local_maps[0][0].sizes() << std::endl; */

	/* std::vector<torch::jit::IValue> inputs; */
	/* inputs.push_back(local_maps); */
	/* torch::Tensor resized_maps = resizer_model.forward(inputs).toTensor(); */
	/* std::cout << "resized_maps: " << resized_maps.sizes() << std::endl; */

	/* torch::Tensor comm_maps = torch::zeros({params.pNumRobots, 32, 32}); */
	/* env.GetAllRobotsCommunicationMaps(comm_maps, 32); */
	/* std::cout << "shape of comm_maps: " << comm_maps.sizes() << std::endl; */

	/* auto sparse_comm_maps = comm_maps.to_sparse(); */
	/* std::cout << sparse_comm_maps << std::endl; */

	/* std::cout << torch::nonzero(comm_maps) - 16 << std::endl; */

	/* torch::Tensor test_tensor = torch::zeros({5}); */
	/* test_tensor[0] = 1; */
	/* test_tensor[1] = 2; */
	/* test_tensor[2] = 3; */
	/* std::cout << test_tensor << std::endl; */
	/* auto test_tensor2 = test_tensor.index({Slice(0, 3)}); */
	/* std::cout << test_tensor2 << std::endl; */
	/* test_tensor2[0] = 10; */
	/* std::cout << test_tensor << std::endl; */


	CCT::GenerateDataset dataset_generator("/root/CoverageControl_ws/data/pure_coverage/data_params.yaml");;


	return 0;

}
