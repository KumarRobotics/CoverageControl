#ifndef COVERAGECONTROL_CNN_BACKBONE_H_
#define COVERAGECONTROL_CNN_BACKBONE_H_

#include <torch/torch.h>

using namespace torch::indexing;

namespace CoverageControlTorch {

	struct CoverageControlCNNImpl : torch::nn::Module {
		int input_dim_ = 4;
		int output_dim_ = 7;
		int num_layers_ = 2;
		int latent_size_ = 8;
		int kernel_size_ = 3;
		int image_size_ = 32;

		torch::nn::ModuleList conv_layers_;
		torch::nn::ModuleList batch_norm_layers_;
		torch::nn::Linear linear_1_;
		torch::nn::Linear linear_2_;

		CoverageControlCNNImpl(int input_dim, int output_dim, int num_layers, int latent_size, int kernel_size, int image_size) : 
			input_dim_(input_dim),
			output_dim_(output_dim),
			num_layers_(num_layers),
			latent_size_(latent_size),
			kernel_size_(kernel_size),
			image_size_(image_size),
			conv_layers_(torch::nn::ModuleList()),
			batch_norm_layers_(torch::nn::ModuleList()),
			linear_1_(nullptr),
			linear_2_(nullptr) {

			std::vector <int> layers_;
			layers_.push_back(input_dim_);
			for(int i = 0; i < num_layers_; ++i) {
				layers_.push_back(latent_size_);
			}

			for(int i = 0; i < num_layers_; ++i) {
				conv_layers_->push_back(register_module("conv" + std::to_string(i),
							torch::nn::Conv2d(torch::nn::Conv2dOptions(layers_[i], layers_[i + 1], 3))));
				batch_norm_layers_->push_back(register_module("batch_norm" + std::to_string(i),
							torch::nn::BatchNorm2d(layers_[i+1])));
			}

			size_t flatten_size = latent_size_ * (image_size_ - num_layers_ * (kernel_size_ - 1)) * (image_size_ - num_layers_ * (kernel_size_ - 1));
			linear_1_ = register_module("linear_1", torch::nn::Linear(flatten_size, latent_size_));
			linear_2_ = register_module("linear_2", torch::nn::Linear(latent_size_, output_dim_));
		} 

		torch::Tensor forward(torch::Tensor x) {
			for(size_t i = 0; i < conv_layers_->size(); ++i) {
				auto batch_norm = (batch_norm_layers_[i].get())->as<torch::nn::BatchNorm2d>();
				auto conv = (conv_layers_[i].get())->as<torch::nn::Conv2d>();
				x = torch::tanh(batch_norm->forward(conv->forward(x)));
				/* std::cout << "x size: " << x.sizes() << std::endl; */
			}
			x = x.flatten(1);
			/* std::cout << "x size: " << x.sizes() << std::endl; */
			x = torch::tanh(linear_1_->forward(x));
			/* std::cout << "x size: " << x.sizes() << std::endl; */
			x = torch::tanh(linear_2_->forward(x));
			/* std::cout << "x size: " << x.sizes() << std::endl; */
			x = torch::tanh(x);
			return x;
		}
	};

	TORCH_MODULE(CoverageControlCNN);


} // namespace CoverageControlTorch

#endif // COVERAGECONTROL_CNN_BACKBONE_H_
