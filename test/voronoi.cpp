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
	Parameters params;
	CoverageSystem env(params, 100, 20);
	std::cout << "Env created" << std::endl;
	auto voronoi_cells = env.LloydOffline();

	return 0;
}
