
#ifndef COVERAGECONTROL_CNN_MODULE_H_
#define COVERAGECONTROL_CNN_MODULE_H_

#include <thread>
#include <string>
#include <yaml-cpp/yaml.h>
#include <torch/torch.h>
#include "cnn_backbone.h"

using namespace torch::indexing;
namespace F = torch::nn::functional;

namespace CoverageControlTorch {


	struct CoverageControlCNNImpl : torch::nn::Module {
		int input_dim_ = 4;
		int output_dim_ = 7;
		int num_layers_ = 2;
		int latent_size_ = 8;
		int kernel_size_ = 3;
		int image_size_ = 32;

		CNNBackbone cnn_backbone_;
		torch::nn::Linear linear_;

		CoverageControlCNNImpl(YAML::Node config) : CoverageControlCNNImpl(
				config["InputDim"].as<int>(),
				config["OutputDim"].as<int>(),
				config["NumLayers"].as<int>(),
				config["LatentSize"].as<int>(),
				config["KernelSize"].as<int>(),
				config["ImageSize"].as<int>()) {}

		CoverageControlCNNImpl(int input_dim, int output_dim, int num_layers, int latent_size, int kernel_size, int image_size) :
			input_dim_(input_dim),
			output_dim_(output_dim),
			num_layers_(num_layers),
			latent_size_(latent_size),
			kernel_size_(kernel_size),
			image_size_(image_size),
			cnn_backbone_(register_module("cnn_backbone", CNNBackbone(input_dim_, output_dim_, num_layers_, latent_size_, kernel_size_, image_size_))),
			linear_(register_module("linear", torch::nn::Linear(2 * output_dim_, output_dim))) {
		}

		torch::Tensor forward(torch::Tensor x) {
			x = cnn_backbone_->forward(x);
			x = linear_->forward(x);
			return x;
		}
	};

	TORCH_MODULE(CoverageControlCNN);

} // CoverageControlTorch
#endif // COVERAGECONTROL_CNN_MODULE_H_
