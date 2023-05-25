#include <torch/torch.h>
#include <iostream>

#include <iomanip>
#include <iostream>
#include <random>

#include <CoverageControl/constants.h>
#include <CoverageControl/typedefs.h>
#include <CoverageControl/bivariate_normal_distribution.h>
#include <CoverageControl/world_idf.h>
#include <CoverageControl/robot_model.h>
#include <CoverageControl/coverage_system.h>

#include <CoverageControlTorch/type_conversions.h>

using namespace torch::indexing;
using namespace CoverageControl;
using namespace CoverageControlTorch;

int main(int argc, char** argv) {
	Parameters params;

	params.pTruncationBND = 10;
	std::unique_ptr <CoverageSystem> env;

	std::vector <Point2> robot_positions;
	robot_positions.push_back(Point2(100, 10));
	robot_positions.push_back(Point2(100, 100));
	robot_positions.push_back(Point2(10, 100));

	WorldIDF world_idf(params);
	world_idf.AddNormalDistribution(BivariateNormalDistribution(Point2(50, 50), 30));

	env = std::make_unique<CoverageSystem> (params, world_idf, robot_positions);

	auto env_map = env->GetWorldIDF();
	auto env_map_tensor = ToTensor(env_map).clone();
	std::cout << env_map_tensor.dtype() << std::endl;

	std::cout << env_map_tensor.sizes() << std::endl;
	print(env_map_tensor[50][50]);

	std::cout << "Saving env_map_tensor to env_map.pt" << std::endl;
	torch::save(env_map_tensor, "env_map.pt");

	torch::Tensor env_map_tensor_loaded;
	torch::load(env_map_tensor_loaded, "env_map.pt");
	print(env_map_tensor_loaded[50][50]);


	std::cout << "Torch indexing and referencing test" << std::endl;
	torch::Tensor test_tensor = torch::zeros({5});
	test_tensor[0] = 1;
	test_tensor[1] = 2;
	test_tensor[2] = 3;
	std::cout << test_tensor << std::endl;
	auto test_tensor2 = test_tensor.index({Slice(0, 3)});
	std::cout << test_tensor2 << std::endl;
	test_tensor2[0] = 10;
	std::cout << test_tensor << std::endl;


	return 0;
}
