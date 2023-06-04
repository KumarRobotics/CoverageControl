#ifndef COVERAGECONTROL_GNN_MODULE_H_
#define COVERAGECONTROL_GNN_MODULE_H_

#include <thread>
#include <string>
#include <yaml-cpp/yaml.h>
#include <torch/torch.h>
#include <torch/script.h>
#include <torchscatter/scatter.h>
#include <torchsparse/sparse.h>
#include "cnn_module.h"

using namespace torch::indexing;
namespace F = torch::nn::functional;

namespace CoverageControlTorch {

	struct CoverageControlGNNImpl : torch::nn::Module {
		YAML::Node config_;
		YAML::Node cnn_config_;

		CoverageControlCNN cnn_module_;
		torch::jit::script::Module gnn_backbone_;

		int input_dim_ = 7;
		int output_dim_ = 2;
		int num_layers_ = 4;
		int latent_size_ = 64;
		int num_hops_ = 2;
		std::vector <torch::Tensor> gnn_parameters_;
		torch::nn::ParameterList gnn_plist_;
		torch::Tensor q;
		torch::nn::Linear linear_1_;

		CoverageControlGNNImpl(YAML::Node const &config) : 
			config_(YAML::Clone(config)),
			cnn_config_(config["CNN"]),
			cnn_module_(CoverageControlCNN(cnn_config_)),
			linear_1_(register_module("linear_1", torch::nn::Linear(64, 2))) {

				torch::load(cnn_module_, config_["CNNModel"]["Dir"].as<std::string>() + config_["CNNModel"]["Model"].as<std::string>());
				register_module("cnn_module", cnn_module_);

				std::string gnn_module_jit = config_["GNNBackBone"]["ScriptName"].as<std::string>();
				gnn_backbone_ = torch::jit::load(gnn_module_jit);
				gnn_backbone_.train();
				/* register_module("gnn_backbone", gnn_backbone_); */
				int param_count = 0;
				/* gnn_parameters_.resize(gnn_backbone_.parameters().size()); */
				for(auto const &params : gnn_backbone_.named_parameters()) {
					/* gnn_parameters_[param_count] = register_parameter(params.name, params.value); */
					/* gnn_plist_.append(register_parameter(params.name, params.value)); */
					std::cout << params.name << " " << params.value.sizes() << std::endl;
					/* gnn_plist_.append(register_parameter(params)); */
					gnn_parameters_.push_back(register_parameter("a" + std::to_string(param_count), params.value));
					++param_count;
					/* parameters_.append(register_parameter(params.name, params.value)); */
					/* parameters_.append(params); */
				}
				std::cout << "No. of params: " << this->parameters().size() << std::endl;
			}

		torch::Tensor forward(torch::Tensor x, torch::Tensor edge_weights) {
			/* std::cout << "edge wt: " << edge_weights.sizes() << std::endl; */
			/* std::cout << "x shape 1: " << x.sizes() << std::endl; */
			x = x.view({-1, x.size(2), x.size(3), x.size(4)});
			/* std::cout << "x shape: " << x.sizes() << std::endl; */
			x = cnn_module_->forward(x);
			/* std::cout << "x shape 2: " << x.sizes() << std::endl; */
			x = torch::leaky_relu(x);
			/* std::cout << "x shape 3: " << x.sizes() << std::endl; */
			/* x = x.view({edge_weights.size(0), -1, 7}); */
			/* std::cout << "x shape 4: " << x.sizes() << std::endl; */
			std::vector<torch::jit::IValue> inputs;
			inputs.push_back(x);
			inputs.push_back(edge_weights);
			x = gnn_backbone_.forward(inputs).toTensor();
			/* std::cout << "x shape 5: " << x.sizes() << std::endl; */
			x = torch::leaky_relu(x);
			/* std::cout << "x shape 6: " << x.sizes() << std::endl; */
			x = linear_1_(x);
			/* std::cout << "x shape 7: " << x.sizes() << std::endl; */

			return x;
		}
	};

	TORCH_MODULE(CoverageControlGNN);


} // CoverageControlTorch
#endif // COVERAGECONTROL_GNN_MODULE_H_
