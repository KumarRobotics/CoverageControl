#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <cstdlib>
#include <fstream>
#include <chrono>
#include <thread>


#include <CoverageControl/constants.h>
#include <CoverageControl/parameters.h>
#include <CoverageControl/typedefs.h>
#include <CoverageControl/bivariate_normal_distribution.h>
#include <CoverageControl/world_idf.h>
#include <CoverageControl/robot_model.h>
#include <CoverageControl/generate_world_map.ch>
#include <CoverageControl/coverage_system.h>
#include <CoverageControl/algorithms/oracle_explore_exploit.h>
#include <CoverageControl/algorithms/oracle_bang_explore_exploit.h>
#include <CoverageControl/algorithms/simul_explore_exploit.h>

using namespace CoverageControl;

int main(int argc, char** argv) {
	Parameters params;
	/* params.pSensorSize = 16; */
	/* params.pMaxRobotSpeed = 10; */
	if (argc == 2) {
		std::string parameter_file = argv[1];
		params = Parameters(parameter_file);
	}
	std::cout << "Processor count: " << std::thread::hardware_concurrency() << std::endl;

	PointVector frontiers;
	int num_robots = 15;
	int num_dists = 10;
	CoverageSystem env(params, num_dists, num_robots);
	OracleSimulExploreExploit oracle(params, num_robots, env);

	std::string dir = "data/oracle/";
	env.PlotWorldMap(dir);
	env.PlotSystemMap(dir, 0);
	int count = 1;
	for(int ii = 1; ii < params.pEpisodeSteps; ++ii) {
		std::cout << "Step: " << ii << std::endl;
		bool cont_flag = oracle.Step();
		auto actions = oracle.GetActions();
		for(int iRobot = 0; iRobot < num_robots; ++iRobot) {
			env.StepAction(iRobot, actions[iRobot]);
		}
		auto robot_status = oracle.GetRobotStatus();
		if(ii%1 == 0) {
			frontiers = oracle.GetFrontiers();
			/* env.PlotFrontiers(dir, count, frontiers); */
			env.PlotSystemMap(dir, count, robot_status);
			++count;
		}
		if(cont_flag == false) {
			break;
		}
	}
	std::cout << "Converged" << std::endl;
	auto robot_status = oracle.GetRobotStatus();
	for(int ii = 0; ii < 20; ++ii) {
		env.PlotSystemMap(dir, count, robot_status);
		++count;
	}

	return 0;
}