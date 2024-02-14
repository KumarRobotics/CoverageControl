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
