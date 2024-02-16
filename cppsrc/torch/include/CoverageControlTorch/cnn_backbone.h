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

#ifndef COVERAGECONTROLTORCH_CNN_BACKBONE_H_
#define COVERAGECONTROLTORCH_CNN_BACKBONE_H_

#include <torch/torch.h>

using namespace torch::indexing;

namespace CoverageControlTorch {

	struct CNNBackboneImpl : torch::nn::Module {
		int input_dim_ = 4;
		int output_dim_ = 7;
		int num_layers_ = 2;
		int latent_size_ = 8;
		int kernel_size_ = 3;
		int image_size_ = 32;

		torch::nn::ModuleList conv_layers_;
		torch::nn::ModuleList batch_norm_layers_;
		torch::nn::Linear linear_1_;

		CNNBackboneImpl() : CNNBackboneImpl(4, 2, 8, 3, 32) {} 
		CNNBackboneImpl(int input_dim, int num_layers, int latent_size, int kernel_size, int image_size) : 
			input_dim_(input_dim),
			num_layers_(num_layers),
			latent_size_(latent_size),
			kernel_size_(kernel_size),
			image_size_(image_size),
			conv_layers_(torch::nn::ModuleList()),
			batch_norm_layers_(torch::nn::ModuleList()),
			linear_1_(nullptr) {

			std::vector <int> layers_;
			layers_.push_back(input_dim_);
			for(int i = 0; i < num_layers_; ++i) {
				layers_.push_back(latent_size_);
			}

			for(int i = 0; i < num_layers_; ++i) {
				conv_layers_->push_back(register_module("conv" + std::to_string(i),
							torch::nn::Conv2d(torch::nn::Conv2dOptions(layers_[i], layers_[i + 1], kernel_size_))));
				batch_norm_layers_->push_back(register_module("batch_norm" + std::to_string(i),
							torch::nn::BatchNorm2d(layers_[i+1])));
			}

			size_t flatten_size = latent_size_ * (image_size_ - num_layers_ * (kernel_size_ - 1)) * (image_size_ - num_layers_ * (kernel_size_ - 1));
			linear_1_ = register_module("linear_1", torch::nn::Linear(flatten_size, latent_size_));
		} 

		torch::Tensor forward(torch::Tensor x) {
			for(size_t i = 0; i < conv_layers_->size(); ++i) {
				auto batch_norm = (batch_norm_layers_[i].get())->as<torch::nn::BatchNorm2d>();
				auto conv = (conv_layers_[i].get())->as<torch::nn::Conv2d>();
				x = torch::leaky_relu(batch_norm->forward(conv->forward(x)));
				/* std::cout << "x size: " << x.sizes() << std::endl; */
			}
			x = x.flatten(1);
			x = torch::leaky_relu(linear_1_->forward(x));
			return x;
		}
	};

	TORCH_MODULE(CNNBackbone);


} // namespace CoverageControlTorch

#endif /* COVERAGECONTROLTORCH_CNN_BACKBONE_H_ */
