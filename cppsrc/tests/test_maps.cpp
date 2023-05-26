#include <torch/torch.h>
#include <iostream>
#include <CoverageControlTorch/coverage_system.h>

using namespace CoverageControl;
int main() {

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

	CoverageControlTorch::CoverageSystem env(params, world_idf, robot_positions);

	/* env.PlotInitMap("./", "init"); */

	auto comm_map = env.GetAllRobotsCommunicationMaps(256);
	std::cout << "Computed communicaiton maps" << std::endl;
	std::cout << comm_map[0][0].to_sparse() << std::endl;
	std::cout << comm_map[0][1].to_sparse() << std::endl;


	torch::Tensor a = torch::rand({5, 2});
	std::cout << a << std::endl;
	torch::Tensor b = a.transpose(1, 0);
	std::cout << b << std::endl;


}
