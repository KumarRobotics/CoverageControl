#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <cstdlib>

#include <CoverageControl/constants.h>
#include <CoverageControl/parameters.h>
#include <CoverageControl/typedefs.h>
#include <CoverageControl/bivariate_normal_distribution.h>
#include <CoverageControl/world_idf.h>
#include <CoverageControl/robot_model.h>
#include <CoverageControl/generate_world_map.ch>
#include <CoverageControl/coverage_system.h>

using namespace CoverageControl;

int main(int argc, char** argv) {
	Parameters params("/home/saurav/CoverageControl_ws/src/CoverageControl/params/parameters.yaml");
	CoverageSystem env(params, 100, 20);
	/* std::cout << "Env created" << std::endl; */
	std::string map_filename = "data/oracle_map";
	std::string gnuplot_script = "src/CoverageControl/scripts/gnuplot/plot_map.gp";

	bool plot_map = false;
	for(int i = 0; i < 100; ++i) {
		std::cout << i << std::endl;
		env.StepDataGenerationLocal(10);
		auto oracle_map = env.GetOracleMap();

		if(plot_map == true) {
			std::stringstream ss;
			ss << std::setw(4) << std::setfill('0') << i;
			std::string s = ss.str();
			std::string imap_name = map_filename + s;
			MapUtils::WriteMap(oracle_map, imap_name + ".dat");
			std::string gnuplot_command = "gnuplot -c " + gnuplot_script + " " + imap_name + " 1 " + std::to_string(params.pResolution) + " " + std::to_string(params.pWorldMapSize * params.pResolution);
			std::system(gnuplot_command.c_str());
		}
	}
	return 0;
}
