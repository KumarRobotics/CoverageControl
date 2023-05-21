#include <iomanip>
#include <iostream>
#include <random>

#include <CoverageControl/constants.h>
#include <CoverageControl/typedefs.h>
#include <CoverageControl/bivariate_normal_distribution.h>
#include <CoverageControl/world_idf.h>
#include <CoverageControl/robot_model.h>
#include <CoverageControl/coverage_system.h>

using namespace CoverageControl;

int main(int argc, char** argv) {
	std::cout << "Coverage Control" << std::endl;
	Parameters params;

	params.pTruncationBND = 10;
	std::unique_ptr <CoverageSystem> env;

	std::vector <Point2> robot_positions;
	robot_positions.push_back(Point2(100, 10));
	robot_positions.push_back(Point2(100, 100));
	robot_positions.push_back(Point2(10, 100));

	WorldIDF world_idf(params);
	world_idf.AddNormalDistribution(BivariateNormalDistribution(Point2(50, 50), 30));
	world_idf.AddNormalDistribution(BivariateNormalDistribution(Point2(1000, 1000), 30));
	world_idf.AddNormalDistribution(BivariateNormalDistribution(Point2(500, 700), Point2(20, 40), 0.8, 1));

	env = std::make_unique<CoverageSystem> (params, world_idf, robot_positions);

	if (argc == 2) {
		std::string parameter_file = argv[1];
		params = Parameters(parameter_file);
		env = std::make_unique<CoverageSystem> (params, params.pNumFeatures, params.pNumRobots);
	}

	if(argc == 4) {
		std::string idf_file = argv[2];
		std::string pos_file = argv[3];
		WorldIDF world_idf(params, idf_file);
		env = std::make_unique<CoverageSystem> (params, world_idf, pos_file);
		params.pNumRobots = env->GetNumRobots();
		params.pNumFeatures = env->GetNumFeatures();
	}

	env->PlotInitMap("data/test", "init_map");

	return 0;
}
