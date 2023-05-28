/** Main program for training a CNN for image classification.
 *
 *  @file: train_cnn.cpp
 */

#include <iostream>
#include <CoverageControlTorch/train_cnn.h>

int main(int argc, char* argv[]) {

	if (argc < 2) {
		std::cout << "Usage: ./train_cnn <dataset_dir>" << std::endl;
		return 1;
	}

	std::string config_file = std::string(argv[1]);
	CoverageControlTorch::TrainCNN train_cnn(config_file);
	train_cnn.Train();

	return 0;
}
