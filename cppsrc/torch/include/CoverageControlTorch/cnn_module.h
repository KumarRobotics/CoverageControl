
#ifndef COVERAGECONTROL_CNN_MODULE_H_
#define COVERAGECONTROL_CNN_MODULE_H_

#include <thread>
#include <string>
#include <torch/torch.h>
#include "cnn_backbone.h"
#include "extern/tomlplusplus/toml.hpp"

using namespace torch::indexing;
namespace F = torch::nn::functional;

namespace CoverageControlTorch {

	struct CoverageControlCNNConfig {
		int pInputDim = 4;
		int pOutputDim = 7;
		int pNumLayers = 2;
		int pLatentSize = 8;
		int pKernelSize = 3;
		int pImageSize = 32;

		CoverageControlCNNConfig(toml::table const &config) {
			if (config.contains("InputDim")) pInputDim = config["InputDim"].value<int>();
			if (config.contains("OutputDim")) pOutputDim = config["OutputDim"].value<int>();
			if (config.contains("NumLayers")) pNumLayers = config["NumLayers"].value<int>();
			if (config.contains("LatentSize")) pLatentSize = config["LatentSize"].value<int>();
			if (config.contains("KernelSize")) pKernelSize = config["KernelSize"].value<int>();
			if (config.contains("ImageSize")) pImageSize = config["ImageSize"].value<int>();
		}

		CoverageControlCNNConfig(int input_dim, int output_dim, int num_layers, int latent_size, int kernel_size, int image_size) : pInputDim(input_dim), pOutputDim(output_dim), pNumLayers(num_layers), pLatentSize(latent_size), pKernelSize(kernel_size), pImageSize(image_size) {}
	};

	struct CoverageControlCNNImpl : torch::nn::Module {

		CoverageControlCNNConfig config_;
		CNNBackbone cnn_backbone_;
		torch::nn::Linear linear_;

		CoverageControlCNNImpl(toml::table &config) :
			config_(config),
			cnn_backbone_(register_module("cnn_backbone", CNNBackbone(config_.pInputDim, config_.pNumLayers, config_.pLatentSize, config_.pKernelSize, config_.pImageSize, config_.pOutputDim))),
			linear_(register_module("linear", torch::nn::Linear(2 * config_.pOutputDim, config_.pOutputDim))) {
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
