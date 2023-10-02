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
	env->WriteEnvironment("datasets/test/pos", "datasets/test/idf");

	CoverageAlgorithm oracle(params, num_robots, *env);

	std::string dir = "datasets/test/maps/";
	env->PlotInitMap(dir, "init_map");
	auto goals = oracle.GetGoals();
	env->PlotMapVoronoi(dir, 0, oracle.GetVoronoi(), oracle.GetGoals());
	for(int ii = 0; ii < params.pEpisodeSteps; ++ii) {
		std::cout << "Step: " << ii << std::endl;
		bool cont_flag = oracle.Step();
		auto actions = oracle.GetActions();
		env->StepActions(actions);
		if(ii%1 == 0) {
			/* env->RecordPlotData(); */
			/* env->PlotMapVoronoi(dir, ii, oracle.GetVoronoi(), oracle.GetGoals()); */
		}
		if(cont_flag == false) {
			break;
		}
	}
	std::cout << "Converged" << std::endl;
	std::cout << "Exploration ratio: " << env->GetExplorationRatio() << " Weighted: " << env->GetWeightedExplorationRatio() << std::endl;
	std::cout << "Coverage objective: " << env->GetObjectiveValue() << std::endl;
	auto zero_actions = PointVector(num_robots, Point2(0,0));

	for(int ii = 0; ii < 90; ++ii) {
		env->StepActions(zero_actions);
		/* env->RecordPlotData(); */
		/* env->PlotMapVoronoi(dir, ii, oracle.GetVoronoi(), oracle.GetGoals()); */
	}

	env->PlotMapVoronoi(dir, 1, oracle.GetVoronoi(), oracle.GetGoals());
	/* env->RenderRecordedMap(dir, "CoverageControl_oracle.mp4"); */
	return 0;
}
