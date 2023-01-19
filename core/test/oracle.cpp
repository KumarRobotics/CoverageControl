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
#include <CoverageControl/oracles/oracle_explore_exploit.h>

using namespace CoverageControl;

int main(int argc, char** argv) {
	Parameters params("/home/saurav/CoverageControl_ws/data/oracle/parameters.yaml");
	int num_robots = 20;
	int num_dists = 20;
	CoverageSystem env(params, num_dists, num_robots);
	OracleExploreExploit oracle(params, num_robots, env);

	std::string map_filename = "data/oracle/oracle_map";
	std::string pos_filename = "data/oracle/pos";
	std::string gnuplot_script = "/home/saurav/CoverageControl_ws/src/CoverageControl/core/scripts/gnuplot/plot_map.gp";

	auto start = std::chrono::steady_clock::now();
		for(int i = 0; i < params.pEpisodeSteps; ++i) {
			std::cout << "iter: " << i << std::endl;
			auto oracle_map = oracle.GetOracleMap();
			std::stringstream ss;
			ss << std::setw(4) << std::setfill('0') << i;
			std::string s = ss.str();
			std::string imap_name = map_filename + s;
			MapUtils::WriteMap(oracle_map, imap_name + ".dat");
			env.WriteRobotPositions(pos_filename);
			std::string gnuplot_command = "gnuplot -c " + gnuplot_script + " " + imap_name + " " + pos_filename + " 1 " + std::to_string(params.pResolution) + " " + std::to_string(params.pWorldMapSize * params.pResolution);
			std::system(gnuplot_command.c_str());
			bool cont_flag = oracle.Step();
			if(cont_flag == false) {
				break;
			}
			/* int exploration_factor = ceil(-12.5 * oracle.GetExplorationRatio() + 22.5); */
			/* if(count == exploration_factor or cont_flag == false) { */
			/* 	count = 0; */
			/* 	cont_flag = oracle.Step(jump_steps, true); */
			/* } else { */
			/* 	cont_flag = oracle.Step(jump_steps, false); */
			/* } */
			/* ++count; */
			/* std::cout << "exp: " << exploration_factor << std::endl; */
		}

		auto end = std::chrono::steady_clock::now();
		std::chrono::duration<double> elapsed_seconds = end-start;
		std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n";
		return 0;
}
