#include <torch/script.h>
#include <torch/torch.h>
#include <torchvision/vision.h>
#include <iostream>

#include <iomanip>
#include <iostream>
#include <random>
#include <filesystem>

#include <CoverageControlTorch/coverage_system.h>
#include <CoverageControlTorch/generate_dataset.h>

using CoverageControl::Point2;
using CoverageControl::BivariateNormalDistribution;
using CoverageControl::WorldIDF;
using namespace torch::indexing;

namespace CC = CoverageControl;
namespace CCT = CoverageControlTorch;

int main(int argc, char** argv) {
	if (argc != 2) {
		std::cout << "Usage: ./data_generation <path_to_yaml>" << std::endl;
		std::cout << "YAML file for data generation is required" << std::endl;
		return 1;
	}

	CCT::GenerateDataset dataset_generator(argv[1]);

	return 0;

}
