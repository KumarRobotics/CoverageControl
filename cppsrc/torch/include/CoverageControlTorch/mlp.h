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

#ifndef COVERAGECONTROLTORCH_MLP_H_
#define COVERAGECONTROLTORCH_MLP_H_

#include <torch/torch.h>

using namespace torch::indexing;
namespace CoverageControlTorch {

	struct MLPImpl : torch::nn::Module {
		int input_size = 256;
		int output_size = 2;
		int layer0_size = 32;
		int layer1_size = 32;
		int output_layer_size = 2;

		torch::nn::Linear linear_layer_0_;
		torch::nn::Linear linear_layer_1_;
		torch::nn::BatchNorm1d batch_norm_layer_0_;
		torch::nn::Linear output_layer_;

		MLPImpl() : 
			linear_layer_0_(register_module("linear_0", torch::nn::Linear(input_size, layer0_size))),
			batch_norm_layer_0_(register_module("batch_norm_0", torch::nn::BatchNorm1d(torch::nn::BatchNorm1dOptions(layer0_size).eps(1e-5).momentum(0.1).affine(true).track_running_stats(true)))),
			linear_layer_1_(register_module("linear_1", torch::nn::Linear(layer0_size, layer1_size))),
			output_layer_(register_module("output_layer", torch::nn::Linear(layer1_size, output_layer_size))) {
			}
			/* batch_norm_layer_0_(register_module("batch_norm_0", torch::nn::BatchNorm1d(layer0_size))), */

		torch::Tensor forward(torch::Tensor x) {
			/* std::cout << "x: " << batch_norm_layer_0_->forward(linear_layer_0_->forward(x)) << std::endl; */ 
			/* x = linear_layer_0_->forward(x); */
			/* std::cout << "sum x l0" << torch::sum(x) << std::endl; */
			/* x = batch_norm_layer_0_->forward(x); */
			/* std::cout << "sum x bn" << torch::sum(x) << std::endl; */
			/* x = torch::relu(x); */
			/* x = linear_layer_1_->forward(x); */
			/* x = output_layer_->forward(x); */
			return output_layer_->forward(linear_layer_1_->forward(torch::relu(batch_norm_layer_0_->forward(linear_layer_0_->forward(x)))));
		}

		void LoadParameters(std::string const &base_dir){
			torch::NoGradGuard no_grad;
			torch::Tensor l0_weight;
			torch::Tensor l0_bias;
			torch::load(l0_weight, base_dir + "/mlp_lin_0.pt");
			torch::load(l0_bias, base_dir + "/mlp_bias_0.pt");
			/* for(auto &p:named_parameters().keys()) { */
			/* 	std::cout << p << std::endl; */
			/* } */
			this->named_parameters()["linear_0.weight"].copy_(l0_weight);
			this->named_parameters()["linear_0.bias"].copy_(l0_bias);

			/* linear_layer_0_->weight = torch::nn::Parameter(l0_weight); */
			/* linear_layer_0_->bias = torch::nn::Parameter(l0_bias); */


			torch::Tensor l1_weight;
			torch::Tensor l1_bias;
			torch::load(l1_weight, base_dir + "/mlp_lin_1.pt");
			torch::load(l1_bias, base_dir + "/mlp_bias_1.pt");
			this->named_parameters()["linear_1.weight"].copy_(l1_weight);
			this->named_parameters()["linear_1.bias"].copy_(l1_bias);
			/* linear_layers_[1]->weight = torch::nn::Parameter(l1_weight); */
			/* linear_layers_[1]->bias = torch::nn::Parameter(l1_bias); */

			torch::Tensor output_weight;
			torch::Tensor output_bias;
			torch::load(output_weight, base_dir + "/outlayer_wts.pt");
			torch::load(output_bias, base_dir + "/outlayer_bias.pt");
			this->named_parameters()["output_layer.weight"].copy_(output_weight);
			this->named_parameters()["output_layer.bias"].copy_(output_bias);
			/* output_layer_->weight = torch::nn::Parameter(output_weight); */
			/* output_layer_->bias = torch::nn::Parameter(output_bias); */

			torch::Tensor bn_weight;
			torch::Tensor bn_bias;
			torch::Tensor bn_running_mean;
			torch::Tensor bn_running_var;
			torch::Tensor bn_num_batches_tracked;
			torch::load(bn_weight, base_dir + "/mlp_norm_0_weight.pt");
			torch::load(bn_bias, base_dir + "/mlp_norm_0_bias.pt");
			torch::load(bn_running_mean, base_dir + "/mlp_norm_0_running_mean.pt");
			torch::load(bn_running_var, base_dir + "/mlp_norm_0_running_var.pt");
			torch::load(bn_num_batches_tracked, base_dir + "/mlp_norm_0_num_batches_tracked.pt");
			this->named_parameters()["batch_norm_0.weight"].copy_(bn_weight);
			this->named_parameters()["batch_norm_0.bias"].copy_(bn_bias);
			this->named_buffers()["batch_norm_0.running_mean"].copy_(bn_running_mean);
			this->named_buffers()["batch_norm_0.running_var"].copy_(bn_running_var);
			this->named_buffers()["batch_norm_0.num_batches_tracked"].copy_(bn_num_batches_tracked);
			/* std::cout << "wts: " << torch::sum(this->named_parameters()["batch_norm_0.weight"]) << std::endl; */
			/* std::cout << "bias: " << torch::sum(this->named_parameters()["batch_norm_0.bias"]) << std::endl; */
			/* std::cout << "mean: " << torch::sum(this->named_buffers()["batch_norm_0.running_mean"]) << std::endl; */
			/* std::cout << "var: " << torch::sum(this->named_buffers()["batch_norm_0.running_var"]) << std::endl; */
		}
	};

	TORCH_MODULE(MLP);


} // namespace CoverageControlTorch

#endif // COVERAGECONTROLTORCH_MLP_H_
