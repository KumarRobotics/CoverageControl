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

#include <torch/script.h>
#include <torch/torch.h>
#include <torchvision/vision.h>
#include <iostream>

#include <iomanip>
#include <iostream>
#include <random>
#include <filesystem>

#include <CoverageControlTorch/coverage_system.h>
#include <CoverageControl/algorithms/lloyd_local_voronoi.h>
#include <CoverageControl/algorithms/lloyd_global_online.h>
#include <CoverageControl/algorithms/oracle_global_offline.h>
#include <CoverageControlTorch/generate_dataset.h>

using CoverageControl::Point2;
using CoverageControl::BivariateNormalDistribution;
using CoverageControl::WorldIDF;
using namespace torch::indexing;

namespace CC = CoverageControl;
namespace CCT = CoverageControlTorch;

int main(int argc, char** argv) {
	if (argc == 1) {
		std::cout << "Usage: ./data_generation <parameters_file>" << std::endl;
		std::cout << "Parameteres file for data generation is required" << std::endl;
		return 1;
	}

	int num_datasets = 1;
	if(argc > 2) {
		num_datasets = std::stoi(argv[2]);
	}

	for(int i = 0; i < num_datasets; i++) {
		std::cout << "Generating dataset " << i << std::endl;
		CCT::GenerateDataset<CC::LloydGlobalOnline> dataset_generator(argv[1], std::to_string(i));
	}

	return 0;

}
