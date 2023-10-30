#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <cstdlib>
#include <fstream>
#include <chrono>
#include <thread>
#include <memory>

#include <CoverageControl/constants.h>
#include <CoverageControl/parameters.h>
#include <CoverageControl/typedefs.h>
#include <CoverageControl/bivariate_normal_distribution.h>
#include <CoverageControl/world_idf.h>
#include <CoverageControl/robot_model.h>
#include <CoverageControl/generate_world_map.ch>
#include <CoverageControl/coverage_system.h>
#include <CoverageControl/algorithms/lloyd_global_online.h>
#include <CoverageControl/algorithms/simul_explore_exploit.h>
#include <CoverageControl/algorithms/lloyd_local_voronoi.h>
#include <CoverageControl/algorithms/oracle_global_offline.h>

using namespace CoverageControl;

typedef LloydGlobalOnline CoverageAlgorithm;
/* typedef LloydLocalVoronoi CoverageAlgorithm; */
/* typedef OracleGlobalOffline CoverageAlgorithm; */
/* typedef OracleSimulExploreExploit CoverageAlgorithm; */

int main(int argc, char** argv) {
	std::cout << "Processor count: " << std::thread::hardware_concurrency() << std::endl;
	Parameters params;
	/* params.pSensorSize = 16; */
	if (argc >= 2) {
		std::string parameter_file = argv[1];
		params = Parameters(parameter_file);
	}
	int num_robots = 50;
	int num_dists = 50;

	std::unique_ptr <CoverageSystem> env;

	if(argc == 4) {
		std::string pos_file = argv[2];
		std::string idf_file = argv[3];
		WorldIDF world_idf(params, idf_file);
		env = std::make_unique<CoverageSystem> (params, world_idf, pos_file);
		num_robots = env->GetNumRobots();
	}
	else {
		env = std::make_unique<CoverageSystem> (params, num_dists, num_robots);
	}

	CoverageAlgorithm oracle(params, num_robots, *env);

	auto goals = oracle.GetGoals();
	std::cout << "Initial objective: " << env->GetObjectiveValue() << std::endl;
	for(int ii = 0; ii < params.pEpisodeSteps; ++ii) {
		bool cont_flag = oracle.Step();
		auto actions = oracle.GetActions();
		env->StepActions(actions);
		if(ii%100 == 0) {
			std::cout << "Step: " << ii << std::endl;
		}
		if(cont_flag == false) {
			break;
		}
	}
	std::cout << "Converged" << std::endl;
	std::cout << "Coverage objective: " << env->GetObjectiveValue() << std::endl;
	auto zero_actions = PointVector(num_robots, Point2(0,0));

	for(int ii = 0; ii < 90; ++ii) {
		env->StepActions(zero_actions);
	}
	return 0;
}