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

#ifndef COVERAGECONTROLTORCH_GNN_BACKBONE_H_
#define COVERAGECONTROLTORCH_GNN_BACKBONE_H_

#include <torch/torch.h>

using namespace torch::indexing;
namespace CoverageControlTorch {

	struct GNNBackBoneImpl : torch::nn::Module {
		int nlayers = 2;
		int K = 3;
		int num_features = 34;
		int latent_size = 256;

		torch::nn::ModuleList Hlk;

		GNNBackBoneImpl() : Hlk(register_module("Hlk", torch::nn::ModuleList())) {
			int l = 0;
			for(int k = 0; k < K; ++k) {
				Hlk->push_back(register_module("lin_" + std::to_string(l) + "_" + std::to_string(k), torch::nn::Linear(torch::nn::LinearOptions(num_features, latent_size).bias(false))));
			}
			Hlk->push_back(register_module("lin_" + std::to_string(l) + "_" + std::to_string(K), torch::nn::Linear(torch::nn::LinearOptions(num_features, latent_size).bias(true))));
			for(l = 1; l < nlayers; ++l) {
				for(int k = 0; k < K; ++k) {
					Hlk->push_back(register_module("lin_" + std::to_string(l) + "_" + std::to_string(k), torch::nn::Linear(torch::nn::LinearOptions(latent_size, latent_size).bias(false))));
				}
				Hlk->push_back(register_module("lin_" + std::to_string(l) + "_" + std::to_string(K), torch::nn::Linear(torch::nn::LinearOptions(latent_size, latent_size).bias(true))));
			}
		}
		torch::Tensor forward(torch::Tensor x, int l, int k) {
			auto layer = (Hlk[l*(K+1) + k].get())->as<torch::nn::Linear>();
			return layer->forward(x);
		}

		void LoadParameters(torch::Device device_, std::string base_dir) {
			torch::NoGradGuard no_grad;
			std::cout << "Loading parameters from: " << base_dir << std::endl;
			for(int l = 0; l < nlayers; ++l) {
				std::string bname = "bias_" + std::to_string(l);
				std::string bfilename = base_dir + bname + ".pt";
				torch::Tensor btensor;
				torch::load(btensor, bfilename);
				btensor = btensor.to(device_);
				// No grad
				btensor.set_requires_grad(false);
				this->named_parameters()["lin_" + std::to_string(l) + "_" + std::to_string(K) + ".bias"].copy_(btensor);

				for(int k = 0; k < K + 1; ++k) {
					std::string name = "lin_" + std::to_string(l) + "_" + std::to_string(k);
					std::string filename = base_dir + name + ".pt";
					torch::Tensor tensor;
					torch::load(tensor, filename);
					tensor = tensor.to(device_);
					// No grad
					tensor.set_requires_grad(false);
					this->named_parameters()[name + ".weight"].copy_(tensor);
				}
			}
		}
	};

	TORCH_MODULE(GNNBackBone);


} // namespace CoverageControlTorch

#endif // COVERAGECONTROLTORCH_GNN_BACKBONE_H_
