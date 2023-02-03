#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <cstdlib>
#include <fstream>
#include <chrono>

#include <CoverageControl/constants.h>
#include <CoverageControl/parameters.h>
#include <CoverageControl/typedefs.h>
#include <CoverageControl/bivariate_normal_distribution.h>
#include <CoverageControl/world_idf.h>
#include <CoverageControl/robot_model.h>
#include <CoverageControl/generate_world_map.ch>
#include <CoverageControl/coverage_system.h>
#include <CoverageControl/oracles/oracle_global_offline.h>

using namespace CoverageControl;

int main(int argc, char** argv) {
	Parameters params;
	/* params.pSensorSize = 16; */
	if (argc == 2) {
		std::string parameter_file = argv[1];
		params = Parameters(parameter_file);
	}

	int num_robots = 15;
	int num_dists = 10;
	CoverageSystem env(params, num_dists, num_robots);
	OracleGlobalOffline oracle(params, num_robots, env);

	std::string dir = "data/test/";
	env.PlotWorldMap(dir);
	auto voronoi = oracle.GetVoronoi();
	auto goals = oracle.GetGoals();
	int count = 0;
	for(int ii = 0; ii < 2000; ++ii) {
		std::cout << "Step: " << ii << std::endl;
		bool cont_flag = oracle.Step();
		auto actions = oracle.GetActions();
		for(size_t iRobot = 0; iRobot < num_robots; ++iRobot) {
			env.StepAction(iRobot, actions[iRobot]);
		}
		if(ii%1 == 0) {
			env.PlotMapVoronoi(dir, count, voronoi, goals);
			++count;
		}
		if(cont_flag == false) {
			break;
		}
	}
	for(int ii = 0; ii < 20; ++ii) {
		env.PlotMapVoronoi(dir, count, voronoi, goals);
		++count;
	}

	return 0;
}
