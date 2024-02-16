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


#include <string>
#include <iostream>
#include <CoverageControlTorch/cnn_module.h>

using namespace CoverageControlTorch;
void LoadDataset(std::string const &data_dir, int const image_size, int const output_dim,  torch::Tensor &maps, torch::Tensor &features, torch::Tensor &features_mean, torch::Tensor &features_std) {
	std::string local_maps_file = data_dir + "/local_maps.pt";
	std::string comm_maps_file = data_dir + "/comm_maps.pt";
	std::string obstacle_maps_file = data_dir + "/obstacle_maps.pt";
	std::string features_file = data_dir + "/normalized_coverage_features.pt";

	torch::Tensor local_maps;
	torch::load(local_maps, local_maps_file);
	local_maps = local_maps.unsqueeze(2).view({-1, 1, image_size, image_size});
	torch::Tensor comm_maps;
	torch::load(comm_maps, comm_maps_file);
	comm_maps = comm_maps.to_dense().view({-1, 2, image_size, image_size});
	torch::Tensor obstacle_maps;
	torch::load(obstacle_maps, obstacle_maps_file);
	obstacle_maps = obstacle_maps.to_dense().unsqueeze(2).view({-1, 1, image_size, image_size});

	torch::load(features, features_file);
	features = features.view({-1, features.size(2)});
	features = features.index({Slice(), Slice(0, output_dim)}).to(torch::kCPU);

	maps = torch::cat({local_maps, comm_maps, obstacle_maps}, 1).to(torch::kCPU);

	torch::load(features_mean, data_dir + "/coverage_features_mean.pt");
	torch::load(features_std, data_dir + "/coverage_features_std.pt");

	std::cout << "maps shape: " << maps.sizes() << std::endl;

}

int main(int argc, char* argv[]) {

	if (argc < 2) {
		std::cout << "Usage: ./test_cnn <parameters_file>" << std::endl;
		return 1;
	}
	std::string config_file = std::string(argv[1]);

	torch::Device device(torch::kCPU);
	if (torch::cuda::is_available()) {
		device = torch::kCUDA;
		std::cout << "Using CUDA" << std::endl;
	}

	YAML::Node config = YAML::LoadFile(config_file);
	auto cnn_config = config["CNN"];

	std::string data_dir = config["DataDir"].as<std::string>();

	CoverageControlCNN model(cnn_config);
	torch::load(model, config["CNNModel"]["Dir"].as<std::string>() + config["CNNModel"]["Model"].as<std::string>());
	model->to(device);

	int image_size = cnn_config["ImageSize"].as<int>();
	int output_dim = cnn_config["OutputDim"].as<int>();

	torch::Tensor maps, features, features_mean, features_std;
	LoadDataset(data_dir, image_size, output_dim, maps, features, features_mean, features_std);
	std::cout << "Loaded dataset" << std::endl;

	maps = maps.to(torch::kCPU);
	torch::Tensor sub_maps = maps.index({Slice(0, 1000)});
	sub_maps = sub_maps.to(device);
	torch::Tensor sub_features = features.index({Slice(0, 1000)});
	std::cout << "submaps created" << std::endl;
	auto pred = model->forward(sub_maps).to(torch::kCPU);
	std::cout << "Pred done" << std::endl;
	auto loss = torch::mse_loss(pred, sub_features);
	std::cout << "Val loss: " << loss << std::endl;
	return 0;
}
