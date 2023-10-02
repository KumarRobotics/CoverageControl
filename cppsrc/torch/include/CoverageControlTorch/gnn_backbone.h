#ifndef COVERAGECONTROL_GNN_BACKBONE_H_
#define COVERAGECONTROL_GNN_BACKBONE_H_

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

#endif // COVERAGECONTROL_GNN_BACKBONE_H_
