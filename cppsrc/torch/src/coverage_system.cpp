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

#include <CoverageControlTorch/coverage_system.h>

namespace CoverageControlTorch {
	void CoverageSystem::LoadGNNParameters(std::string const &base_dir) {
		int num_layers = 5;
		int K = 3;
		for(int l = 0; l < num_layers; ++l) {
			std::string bname = "bias_" + std::to_string(l);
			std::string bfilename = base_dir + bname + ".pt";
			torch::Tensor btensor;
			torch::load(btensor, bfilename);
			gnn_parameters_biases_.push_back(btensor);
			std::vector<torch::Tensor> weights;
			for(int k = 0; k < K + 1; ++k) {
				weights.clear();
				std::string name = "lin_" + std::to_string(l) + "_" + std::to_string(k);
				std::string filename = base_dir + name + ".pt";
				torch::Tensor tensor;
				torch::load(tensor, filename);
				weights.push_back(tensor);
			}
			gnn_parameters_weights_.push_back(weights);
		}
	}

	void CoverageSystem::ComputeAdjacencyMatrix() {
		int num_nodes = num_robots_;
		torch::Tensor adj_mat_ = torch::zeros({num_nodes, num_nodes});
		for(int i = 0; i < num_nodes; ++i) {
			for(int j = 0; j < num_nodes; ++j) {
				if(i == j) {
					continue;
				}
				if((robot_positions_[i] - robot_positions_[j]).norm() < params_.pCommunicationRange) {
					adj_mat_.index_put_({i, j}, 1);
				}
			}
		}
	}

	torch::Tensor CoverageSystem::GNNInference() {

		std::vector<torch::Tensor> X;
		std::vector<torch::Tensor> Y;
		std::vector<torch::Tensor> Z;
		int num_nodes = num_robots_;
		int nlayers = 5;
		int K = 3;
		int num_features = 34;
		int latent_size = 256;
		for(int l = 0; l < nlayers + 1; ++l) {
			if(l == 0) {
				X.push_back(torch::zeros({num_nodes, num_features}));
				Y.push_back(torch::zeros({num_nodes, K + 1, latent_size}));
				Z.push_back(torch::zeros({num_nodes, latent_size}));
				continue;
			}
			if(l == 1) {
				X.push_back(torch::zeros({num_nodes, latent_size}));
				Y.push_back(torch::zeros({num_nodes, K + 1, features}));
				Z.push_back(torch::zeros({num_nodes, latent_size}));
				continue;
			}
			X.push_back(torch::zeros({num_nodes, latent_size}));
			Y.push_back(torch::zeros({num_nodes, K + 1, latent_size}));
			Z.push_back(torch::zeros({num_nodes, latent_size}));
		}

		for(int i = 0; i < num_nodes; ++i) {
			X[0].index_put_({i}, sensor_information_[i]);
			Z[0].index_put_({i}, sensor_information_[i]);
			Y[0].index_put_({i, 0}, sensor_information_[i]);
		}

		for(int l = 1; l < nlayers + 1; ++l) {
			std::cout << "layer: " << l << std::endl;
			for(int k = 0; k < K + 1; ++k) {
				std::cout << "k: " << k << std::endl;
				for(int i = 0; i < num_nodes; ++i) {
					if(k == 0) {
						Y[l].index_put_({i, 0}, X[l-1].index({i}));
						Z[l].index_put_({i}, torch::matmul(Y[l].index({i, 0}), gnn_parameters_weights_[l-1][0].t()));
						continue;
					}
					torch::Tensor deg_i = torch::sum(adj_mat_[i]);
					for(int j = 0; j < num_nodes; ++j) {
						if(i == j) {
							continue;
						}
						if(adj_mat_[i][j] == 0) {
							continue;
						}
						torch::Tensor deg_j = torch::sum(adj_mat_[j]);
						Y[l].index_put_({i, k}, Y[l].index({i, k}) + Y[l].index({j, k-1}) / torch::sqrt(deg_i * deg_j));
					}
					Z[l].index_put_({i}, Z[l].index({i}) + torch::matmul(Y[l].index({i, k}), gnn_parameters_weights_[l-1][k].t()));
				}
			}
			for(int i = 0; i < num_nodes; ++i) {
				Z[l].index_put_({i}, Z[l].index({i}) + gnn_parameters_biases_[l-1]);
				X[l].index_put_({i}, torch::relu(Z[l].index({i})));
			}
		}
		return X[nlayers];
	}

	PointVector CoverageSystem::GNNCNNInference() {
		ComputeAdjacencyMatrix();
		UpdateSensorInformation();
		torch::Tensor cnn_input = sensor_information_.clone();
		cnn_input = cnn_input.view({-1, cnn_input.size(-3), cnn_input.size(-2), cnn_input.size(-1)});
		cnn_output_ = cnn_backbone_.forward(cnn_input);
		cnn_output_ = cnn_output_.view({num_robots_, 32});
		torch::Tensor robot_positions = ToTensor(GetRobotPositions());
		torch::Tensor robot_positions_scaled = robot_positions / env_resolution_;
		robot_positions = (robot_positions + params_.pWorldMapSize/2.) / params_.pWorldMapSize;
		// Concat robot positions with cnn output
		torch::Tensor gnn_input = torch::cat({robot_positions, cnn_output_}, 1);
		torch::Tensor gnn_output = GNNInference();
		torch::Tensor controls = GNNPostProcessing(gnn_output);
	}
}
