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
	auto env_map_tensor = EigenToLibTorch(env_map);
	std::cout << env_map_tensor.dtype() << std::endl;

	print(env_map_tensor[50][50]);

	return 0;
}
