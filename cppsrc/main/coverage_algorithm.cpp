/*
 * This file is part of the CoverageControl library
 *
 * Author: Saurav Agarwal
 * Contact: sauravag@seas.upenn.edu, agr.saurav1@gmail.com
 * Repository: https://github.com/KumarRobotics/CoverageControl
 *
 * The CoverageControl library is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
 *
 * The CoverageControl library is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with CoverageControl library. If not, see <https://www.gnu.org/licenses/>.
 */

/*!
 * \file coverage_algorithm.cpp
 * \brief Program to test the CVT-based (Lloyd's) coverage control algorithms
 *
 * This program is used to execute CVT-based (Lloyd's) coverage control algorithms.
 * The environment can be initialized with default parameters.
 * The program can also take in a parameter file, a position file and an IDF file.
 * The position file contains the initial positions of the robots and the IDF file contains the location of the features of interest.
 * The program then runs the coverage control algorithm and outputs the final objective value.
 * ```bash
 * ./coverage_algorithm [parameter_file] [<position_file> <idf_file>]
 * ```
 *
 */

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
#include <CoverageControl/generate_world_map.h>
#include <CoverageControl/coverage_system.h>
#include <CoverageControl/algorithms/clairvyont_cvt.h>
#include <CoverageControl/algorithms/simul_explore_exploit.h>
#include <CoverageControl/algorithms/decentralized_cvt.h>
#include <CoverageControl/algorithms/centralized_cvt.h>
#include <CoverageControl/algorithms/oracle_global_offline.h>

using namespace CoverageControl;

typedef ClairvyontCVT CoverageAlgorithm;
/* typedef CentralizedCVT CoverageAlgorithm; */
/* typedef DecentralizedCVT CoverageAlgorithm; */
/* typedef DecentralizedCVT CoverageAlgorithm; */
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

	std::unique_ptr <CoverageSystem> env;

	if(argc == 3) {
		std::cerr << "Please provide both position and IDF files" << std::endl;
		std::cerr << "Usage: ./coverage_algorithm [parameter_file] [<position_file> <idf_file>]" << std::endl;
		return 1;
	}

	else
	if(argc == 4) {
		std::string pos_file = argv[2];
		std::string idf_file = argv[3];
		WorldIDF world_idf(params, idf_file);
		env = std::make_unique<CoverageSystem> (params, world_idf, pos_file);
	}
	else {
		env = std::make_unique<CoverageSystem> (params);
	}

	int num_robots = env->GetNumRobots();

	CoverageAlgorithm oracle(params, num_robots, *env);

	auto goals = oracle.GetGoals();
	std::cout << "Initial objective: " << env->GetObjectiveValue() << std::endl;
	for(int ii = 0; ii < params.pEpisodeSteps; ++ii) {
		bool cont_flag = oracle.Step();
		auto actions = oracle.GetActions();
		if(env->StepActions(actions)) {
			std::cout << "Invalid action" << std::endl;
			break;
		}
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
		if(env->StepActions(zero_actions)) {
			std::cout << "Invalid action" << std::endl;
			break;
		}
	}
	return 0;
}
