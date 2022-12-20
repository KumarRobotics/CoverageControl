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
	for(int j = 0; j < 100; ++j) {
		CoverageSystem env(params, 100, 20);
		std::cout << "Env created" << std::endl;
		/* for(int i = 0; i < params.pEpisodeSteps; ++i) { */
		/* 	/1* std::cout << "steps: " << i << std::endl; *1/ */
		/* 	env.StepOracle(); */
		/* } */
	}

	return 0;
}
