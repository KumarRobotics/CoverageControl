
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

/* typedef LloydGlobalOnline CoverageAlgorithm; */
typedef LloydLocalVoronoi CoverageAlgorithm;
/* typedef OracleGlobalOffline CoverageAlgorithm; */
/* typedef OracleSimulExploreExploit CoverageAlgorithm; */

int main(int argc, char** argv) {
	std::cout << "Processor count: " << std::thread::hardware_concurrency() << std::endl;
	Parameters params;
	/* params.pSensorSize = 16; */
	if (argc == 2) {
		std::string parameter_file = argv[1];
		params = Parameters(parameter_file);
	}
	int num_robots = 50;
	int num_dists = 50;

	std::fstream file_comp;
	file_comp.open("data/test/comp.txt", std::ios::out);


	for(int iEnv = 0; iEnv < 1000; ++iEnv) {
		std::cout << "Environment: " << iEnv << std::endl;
		std::unique_ptr <CoverageSystem> originial_env;
		originial_env = std::make_unique<CoverageSystem> (params, num_dists, num_robots);
		originial_env->WriteEnvironment("data/test/pos", "data/test/idf");
		for(int iType = 0; iType < 2; ++iType) {
			std::cout << "Type: " << iType << std::endl;
			std::unique_ptr <CoverageSystem> env;
			WorldIDF world_idf(params, "data/test/idf");
			env = std::make_unique<CoverageSystem> (params, world_idf, "data/test/pos");
			CoverageAlgorithm oracle(params, num_robots, *env);

			std::string dir = "data/test/";
			for(int ii = 0; ii < params.pEpisodeSteps; ++ii) {
				std::cout << "Step: " << ii << std::endl;
				bool cont_flag = false;
				if(iType == 0) {
					cont_flag = oracle.Step(true);
				} else {
					cont_flag = oracle.Step(false);
				}
				auto actions = oracle.GetActions();
				env->StepActions(actions);
				if(cont_flag == false) {
					break;
				}
			}
			std::cout << "Converged" << std::endl;
			std::cout << "Exploration ratio: " << env->GetExplorationRatio() << " Weighted: " << env->GetWeightedExplorationRatio() << std::endl;
			std::cout << "Coverage objective: " << env->GetObjectiveValue() << std::endl;
			file_comp << env->GetObjectiveValue();
			if(iType == 0) {
				file_comp << " ";
			} else {
				file_comp << std::endl;
			}
		}
	}
	file_comp.close();

	return 0;
}
