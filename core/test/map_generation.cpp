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
	if (argc == 2) {
		std::string parameter_file = argv[1];
		params = Parameters(parameter_file);
	}

	int num_robots = 50;
	int num_dists = 50;

	std::unique_ptr <CoverageSystem> env;

	if(argc == 4) {
		std::string idf_file = argv[2];
		std::string pos_file = argv[3];
		WorldIDF world_idf(params, idf_file);
		env = std::make_unique<CoverageSystem> (params, world_idf, pos_file);
		num_robots = env->GetNumRobots();
	}
	else {
		env = std::make_unique<CoverageSystem> (params, num_dists, num_robots);
	}
	env->PlotInitMap("data/test", "init_map");

	return 0;
}
