/** Main program for training a GNN
 *
 *  @file: train_gnn.cpp
 */

#include <iostream>
#include <CoverageControlTorch/train_gnn.h>

int main(int argc, char* argv[]) {

	if (argc < 2) {
		std::cout << "Usage: ./train_gnn <yaml>" << std::endl;
		return 1;
	}

	std::string config_file = std::string(argv[1]);
	CoverageControlTorch::TrainGNN train_gnn(config_file);
	train_gnn.Train();

	return 0;
}
