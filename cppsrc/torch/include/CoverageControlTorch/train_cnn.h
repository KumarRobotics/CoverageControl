/** This file contains the declaration of the class TrainCNN using Torch C++ API.
 * The class TrainCNN takes local maps, communication maps, and obstacles maps as input, and
 * predicts the voronoi coverage features.
 *
 **/

#ifndef COVERAGECONTROL_TRAIN_CNN_H_
#define COVERAGECONTROL_TRAIN_CNN_H_


#include <iostream>
#include <string>
#include <vector>
#include <filesystem>
#include <yaml-cpp/yaml.h>
#include <torch/torch.h>

#include "cnn_backbone.h"

using namespace torch::indexing;
namespace F = torch::nn::functional;

namespace CoverageControlTorch {

	class TrainCNN {
		private:
			torch::Tensor maps_;
			torch::Tensor features_;
			torch::Device device_ = torch::kCPU;
			YAML::Node config_;
			YAML::Node cnn_config_;
			std::string data_dir_;
			size_t batch_size_ = 64;
			size_t num_epochs_ = 10;
			float learning_rate_ = 0.001;
			float weight_decay_ = 0.0001;
			int image_size_ = 32;

			std::shared_ptr<torch::optim::Adam> optimizer_;
		public:

			TrainCNN(std::string const &config_file) {
				if (torch::cuda::is_available()) {
					device_ = torch::kCUDA;
					std::cout << "Using CUDA" << std::endl;
				}
				LoadConfigs(config_file);
			}

			/** Train CNN model.
			 * @param dataset_dir: the directory of the dataset.
			 * @param num_layers: the number of convolutional layers.
			 * @param num_epochs: the number of epochs.
			 * @param learning_rate: the learning rate.
			 * @param batch_size: the batch size.
			 **/
			void Train() {
				LoadDataset();

				CoverageControlCNN model(
						cnn_config_["InputDim"].as<int>(),
						cnn_config_["OutputDim"].as<int>(),
						cnn_config_["NumLayers"].as<int>(),
						cnn_config_["LatentSize"].as<int>(),
						cnn_config_["KernelSize"].as<int>(),
						image_size_);


				model->to(device_);

				optimizer_ = std::make_shared<torch::optim::Adam>(
						model->parameters(),
						torch::optim::AdamOptions(learning_rate_).weight_decay(weight_decay_));

				size_t dataset_size = maps_.size(0);
				for (size_t epoch = 1; epoch < num_epochs_ + 1; ++epoch) {
					for (size_t i = 0; i < dataset_size; i += batch_size_) {
						auto loss = TrainOneBatch(model, i);
						std::cout << "Epoch: " << epoch << ", Batch: " << i << ", Loss: " << loss << std::endl;
					}
				}
				maps_ = maps_.to(device_);
				auto pred = model->forward(maps_).to(torch::kCPU);
				features_ = features_.to(torch::kCPU);
				// Compute loss individually for each feature in features
				auto loss = torch::mse_loss(pred, features_);
				std::cout << "Final loss: " << loss.item<float>() << std::endl;
				auto loss_vec = torch::norm(pred - features_, 2, 0).to(torch::kCPU);
				std::cout << "Loss vector: " << loss_vec << std::endl;
				std::cout << "Max of feature 0 true: " << features_.index({Slice(), 0}).max() << std::endl;
				std::cout << "Max of feature 0 pred: " << pred.index({Slice(), 0}).max() << std::endl;
				std::cout << "Max of feature 1 true: " << features_.index({Slice(), 1}).max() << std::endl;
				std::cout << "Max of feature 1 pred: " << pred.index({Slice(), 1}).max() << std::endl;

			}

			float TrainOneBatch(CoverageControlCNN &model, size_t batch_idx) {
				torch::Tensor batch = maps_.index({Slice(batch_idx, batch_idx + batch_size_)});
				batch = batch.to(device_);
				auto x = model->forward(batch);

				// Backward and optimize
				optimizer_->zero_grad();
				torch::Tensor batch_features = features_.index({Slice(batch_idx, batch_idx + batch_size_)}).to(device_);
				auto loss = torch::mse_loss(x, batch_features);
				loss.backward();
				optimizer_->step();

				return loss.item<float>();

			}


			/** Function to load the dataset from the dataset directory.
			 * @param dataset_dir: the directory of the dataset.
			 **/
			void LoadDataset() {
				std::string local_maps_file = data_dir_ + "/local_maps.pt";
				std::string comm_maps_file = data_dir_ + "/comm_maps.pt";
				std::string obstacle_maps_file = data_dir_ + "/obstacle_maps.pt";
				std::string features_file = data_dir_ + "/normalized_coverage_features.pt";

				torch::Tensor local_maps;
				torch::load(local_maps, local_maps_file);
				local_maps = local_maps.unsqueeze(2).view({-1, 1, image_size_, image_size_});
				torch::Tensor comm_maps;
				torch::load(comm_maps, comm_maps_file);
				comm_maps = comm_maps.to_dense().view({-1, 2, image_size_, image_size_});
				torch::Tensor obstacle_maps;
				torch::load(obstacle_maps, obstacle_maps_file);
				obstacle_maps = obstacle_maps.to_dense().unsqueeze(2).view({-1, 1, image_size_, image_size_});

				torch::load(features_, features_file);
				features_ = features_.view({-1, features_.size(2)});
				int output_dim = config_["CNN"]["OutputDim"].as<int>();
				features_ = features_.index({Slice(), Slice(0, output_dim)});

				maps_ = torch::cat({local_maps, comm_maps, obstacle_maps}, 1);
				std::cout << "maps shape: " << maps_.sizes() << std::endl;

			}

			void LoadConfigs(std::string const &config_file) {
				std::cout << "Using config file: " << config_file << std::endl;
				// Check if config_file exists
				if(not std::filesystem::exists(config_file)) {
					throw std::runtime_error("Could not open config file: " + config_file);
				}
				config_ = YAML::LoadFile(config_file);
				data_dir_ = config_["pDataDir"].as<std::string>();
				batch_size_ = config_["BatchSize"].as<size_t>();
				num_epochs_ = config_["NumEpochs"].as<size_t>();
				learning_rate_ = config_["LearningRate"].as<float>();
				weight_decay_ = config_["WeightDecay"].as<float>();

				cnn_config_ = config_["CNN"]	;
				image_size_ = cnn_config_["ImageSize"].as<int>();
			}

	};

} // namespace CoverageControlTorch

#endif //COVERAGECONTROL_TRAIN_CNN_H_
