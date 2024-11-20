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

#ifndef COVERAGECONTROLTORCH_GENERATE_DATASET_H_
#define COVERAGECONTROLTORCH_GENERATE_DATASET_H_

#include <iostream>
#include <fstream>
#include <memory>
#include <chrono>
#include <ctime>
#include <CoverageControl/coverage_system.h>

#include <torch/script.h>
#include <torch/torch.h>

#include "extern/tomlplusplus/toml.hpp"

using namespace torch::indexing;
typedef long int T_idx_t;
namespace F = torch::nn::functional;
/* typedef CoverageControl::LloydGlobalOnline CoverageAlgorithm; */
/* typedef CoverageControl::OracleGlobalOffline CoverageAlgorithm; */

#include "coverage_system.h"

namespace CoverageControlTorch {

	template <class CoverageAlgorithm>
	class GenerateDataset {

		private:
			toml::table config_;
			std::string data_dir_;

			CoverageControl::Parameters env_params_;
			std::shared_ptr <CoverageSystem> env_;
			std::shared_ptr <CoverageAlgorithm> alg_;

			int num_robots_;
			size_t dataset_size_;
			size_t dataset_count_ = 0;
			int map_size_ = 32;
			size_t every_num_step_ = 1;
			size_t env_count_ = 0;
			float converged_data_ratio_ = 0.0;
			size_t trigger_size_ = std::numeric_limits<size_t>::max();
			size_t trigger_count_ = 0;
			size_t trigger_start_idx_ = 0;
			float comm_range_ = 256;
			float env_resolution_ = 1;
			size_t num_non_converged_env_ = 0;
			std::string data_dir_append_ = "";
			std::string data_folder_;

			torch::DeviceType device_type_ = torch::kCPU;
			torch::Device device_ = device_type_;

			torch::Tensor actions_;
			torch::Tensor robot_positions_;
			torch::Tensor raw_local_maps_;
			torch::Tensor local_maps_;
			torch::Tensor raw_obstacle_maps_;
			torch::Tensor obstacle_maps_;
			torch::Tensor comm_maps_;
			torch::Tensor coverage_features_;

			torch::jit::script::Module torchvision_resizer_;

		public:
			GenerateDataset(std::string const &config_file, std::string data_dir_append = "") {
				auto start_time = std::chrono::system_clock::now();
				data_dir_append_ = data_dir_append;

				LoadConfigs(config_file);
				auto toml_NumDataset = config_["NumDataset"].value<int>();
				if(toml_NumDataset) {
					dataset_size_ = toml_NumDataset.value();
				} else {
					throw std::runtime_error("Could not find NumDataset in config file: " + config_file);
				}
				auto toml_converged_data_ratio = config_["ConvergedDataRatio"].value<float>();
				if(toml_converged_data_ratio) {
					converged_data_ratio_ = toml_converged_data_ratio.value_or(0.0);
				}

				num_robots_ = env_params_.pNumRobots;
				comm_range_ = (float)env_params_.pCommunicationRange;
				env_resolution_ = (float)env_params_.pResolution;

				map_size_ = config_["MapSize"].value<int>().value_or(map_size_);
				every_num_step_ = config_["EveryNumStep"].value<size_t>().value_or(every_num_step_);
				trigger_size_ = config_["TriggerPostProcessing"].value<size_t>().value_or(trigger_size_);
				if(trigger_size_ == 0 or trigger_size_ > dataset_size_ ) {
					trigger_size_ = dataset_size_;
				}

				if(torch::cuda::is_available()) {
					device_type_ = torch::kCUDA;
				} else {
					device_type_ = torch::kCPU;
				}
				device_ = torch::Device(device_type_);

				actions_ = torch::empty({dataset_size_, num_robots_, 2});
				robot_positions_ = torch::empty({dataset_size_, num_robots_, 2});
				raw_local_maps_ = torch::empty({trigger_size_, num_robots_, env_params_.pLocalMapSize, env_params_.pLocalMapSize});
				raw_obstacle_maps_ = torch::empty({trigger_size_, num_robots_, env_params_.pLocalMapSize, env_params_.pLocalMapSize});
				local_maps_ = torch::zeros({dataset_size_, num_robots_, map_size_, map_size_});
				obstacle_maps_ = torch::zeros({dataset_size_, num_robots_, map_size_, map_size_});
				comm_maps_ = torch::zeros({dataset_size_, num_robots_, 2, map_size_, map_size_});
				coverage_features_ = torch::empty({dataset_size_, num_robots_, 7});
				PrintTensorSizes(std::cout);
				std::ofstream file;
				std::time_t start_time_t = std::chrono::system_clock::to_time_t(start_time);
				file.open(data_folder_ + "/metrics.txt");
				file << "Start time: " << std::ctime(&start_time_t) << std::endl;
				PrintTensorSizes(file);
				file.close();
				Run();
				auto end_time = std::chrono::system_clock::now();
				std::chrono::duration<double> elapsed_seconds = end_time - start_time;
				std::time_t end_time_t = std::chrono::system_clock::to_time_t(end_time);
				file.open(data_folder_ + "/metrics.txt", std::ios_base::app);
				file << std::endl;
				file << "Number of environments: " << env_count_ << std::endl;
				file << "Number of non-converged enironments: " << num_non_converged_env_ << std::endl;
				file << std::endl;
				file << "Finished computation at " << std::ctime(&end_time_t)
					<< "elapsed time: " << elapsed_seconds.count()/3600 << " hrs"
					<< std::endl;
				file.close();
			}

			void Run() {
				num_non_converged_env_ = 0;
				while(dataset_count_ < dataset_size_) {
					env_ = std::make_shared <CoverageSystem>(env_params_, env_params_.pNumGaussianFeatures, env_params_.pNumRobots);
					alg_ = std::make_shared <CoverageAlgorithm>(env_params_, num_robots_, *env_);

					++env_count_;
					size_t num_steps = 0;
					bool converged = false;
					while(num_steps < env_params_.pEpisodeSteps and not converged and dataset_count_ < dataset_size_) {
						if(num_steps % every_num_step_ == 0) {
							converged = StepWithSave();
						} else {
							converged = StepWithoutSave();
						}
						++num_steps;
					}
					if(num_steps == env_params_.pEpisodeSteps) {
						++num_non_converged_env_;
						std::cout << "num_non_converged_env: " << num_non_converged_env_ << std::endl;
					}
					size_t num_converged_data = std::ceil(converged_data_ratio_ * num_steps / every_num_step_);
					size_t converged_data_count = 0;
					while(converged_data_count < num_converged_data and dataset_count_ < dataset_size_) {
						converged = StepWithSave();
						++converged_data_count;
					}
				}
				/* ProcessCommunicationMaps(); */
				std::cout << "Saving dataset to " << data_folder_ << std::endl;
				// Create data subfolder if it does not exists
				if(dataset_size_%trigger_size_ != 0) {
					ProcessLocalMaps();
				}
				std::cout << "num_non_converged_env: " << num_non_converged_env_ << std::endl;
				ProcessEdgeWeights();
				SaveDataset();
			}

		private:
			inline auto GetTensorByteSizeMB(torch::Tensor const &tensor) {
				return tensor.numel() * torch::elementSize(torch::typeMetaToScalarType(tensor.dtype()))/1000000.;
			}

			bool StepWithoutSave() {
				bool converged = not alg_->Step();
				auto actions = alg_->GetActions();
				auto error_flag = env_->StepActions(actions);
				return converged or error_flag;
			}

			bool StepWithSave() {
				bool converged = not alg_->Step();
				auto actions = alg_->GetActions();
				actions_[dataset_count_] = ToTensor(actions);
				robot_positions_[dataset_count_] = ToTensor(env_->GetRobotPositions());
				coverage_features_[dataset_count_] = ToTensor(env_->GetLocalVoronoiFeatures());
				raw_local_maps_[trigger_count_] = env_->GetAllRobotsLocalMaps().clone();
				comm_maps_[dataset_count_] = env_->GetAllRobotsCommunicationMaps(map_size_).clone();
				raw_obstacle_maps_[trigger_count_] = env_->GetAllRobotsObstacleMaps().clone();
				/* for(size_t i = 0; i < num_robots_; ++i) { */
				/* 	raw_local_maps_[trigger_count_][i] = ToTensor(env_->GetRobotLocalMap(i)); */
				/* } */
				++dataset_count_;
				if(dataset_count_ % 100 == 0) {
					std::cout << dataset_count_ << " " << env_count_ << std::endl;
				}
				++trigger_count_;
				if(trigger_count_ == trigger_size_) {
					trigger_count_ = 0;
					ProcessLocalMaps();
				}
				auto error_flag = env_->StepActions(actions);
				return converged or error_flag;
			}

			void ProcessEdgeWeights() {
				float onebyexp = expf(-1);
				robot_positions_.to(device_);
				torch::Tensor  pairwise_dist_matrices= torch::cdist(robot_positions_, robot_positions_, 2);

				torch::Tensor edge_weights = torch::exp(-(pairwise_dist_matrices.square())/(comm_range_*comm_range_)).to(torch::kCPU);
				F::threshold(edge_weights, F::ThresholdFuncOptions(onebyexp, 0).inplace(true));
				torch::Tensor diagonal_mask = torch::eye(edge_weights.size(1)).repeat({edge_weights.size(0), 1, 1}).to(torch::kBool);
				edge_weights.masked_fill_(diagonal_mask, 0);
				edge_weights.to(torch::kCPU);
				if(config_["SaveAsSparseQ"].value<bool>().value_or(true)) {
					torch::save(edge_weights.to_sparse(), data_folder_ + "edge_weights.pt");
				} else {
					torch::save(edge_weights, data_folder_ + "edge_weights.pt");
				}
			}


			/* void ProcessCommunicationMaps() { */
			/* 	for (T_idx_t b_idx = 0; b_idx < dataset_size_; ++b_idx) { */
			/* 		comm_maps_[b_idx] = env_->GetAllRobotsCommunicationMaps(map_size_).clone(); */
			/* 	} */
			/* 	/1* comm_maps.to(device_); *1/ */
			/* } */

			void ProcessLocalMaps() {
				std::cout << "Processing local maps" << std::endl;
				if(trigger_start_idx_ > dataset_size_ - 1) { return; }

				size_t trigger_end_idx = trigger_start_idx_ + trigger_size_;
				if(trigger_end_idx > dataset_size_) {
					trigger_end_idx = dataset_size_;
				}

				torch::Tensor raw_local_maps = raw_local_maps_.index({Slice(0, trigger_end_idx - trigger_start_idx_), Slice(), Slice(), Slice()});
				raw_local_maps.to(device_);
				torch::Tensor local_maps = raw_local_maps.view({-1, env_params_.pLocalMapSize, env_params_.pLocalMapSize});
				torch::Tensor output = torchvision_resizer_.forward({local_maps}).toTensor().to(torch::kCPU);
				torch::Tensor transformed_local_maps = output.view({-1, num_robots_, map_size_, map_size_});
				local_maps_.index_put_({Slice(trigger_start_idx_, trigger_end_idx)}, transformed_local_maps.clone());

				torch::Tensor raw_obstacle_maps = raw_obstacle_maps_.index({Slice(0, trigger_end_idx - trigger_start_idx_), Slice(), Slice(), Slice()});
				raw_obstacle_maps.to(device_);
				torch::Tensor obstacle_maps = raw_obstacle_maps.view({-1, env_params_.pLocalMapSize, env_params_.pLocalMapSize});
				torch::Tensor output_obstacle_maps = torchvision_resizer_.forward({obstacle_maps}).toTensor().to(torch::kCPU);
				torch::Tensor transformed_obstacle_maps = output_obstacle_maps.view({-1, num_robots_, map_size_, map_size_});
				obstacle_maps_.index_put_({Slice(trigger_start_idx_, trigger_end_idx)}, transformed_obstacle_maps.clone());

				trigger_start_idx_ = trigger_end_idx;
				raw_local_maps.to(torch::kCPU);
				raw_obstacle_maps.to(torch::kCPU);
			}

			void SaveDataset() {

				torch::save(robot_positions_, data_folder_ + "/robot_positions.pt");
				torch::save(local_maps_, data_folder_ + "/local_maps.pt");

				torch::save(actions_, data_folder_ + "/actions.pt");
				torch::save(coverage_features_, data_folder_ + "/coverage_features.pt");

				if(config_["SaveAsSparseQ"].value<bool>().value_or(true)) {
					torch::save(comm_maps_.to_sparse(), data_folder_ + "/comm_maps.pt");
					torch::save(obstacle_maps_.to_sparse(), data_folder_ + "/obstacle_maps.pt");
				} else {
					torch::save(comm_maps_, data_folder_ + "/comm_maps.pt");
					torch::save(obstacle_maps_, data_folder_ + "/obstacle_maps.pt");
				}
				if(config_["NormalizeQ"].value<bool>().value_or(true)) {
					torch::Tensor actions_mean = at::mean(actions_.view({-1,2}), 0);
					torch::Tensor actions_std = at::std(actions_.view({-1,2}), 0);
					torch::Tensor normalized_actions = (actions_ - actions_mean)/actions_std;
					torch::save(actions_mean, data_folder_ + "/actions_mean.pt");
					torch::save(actions_std, data_folder_ + "/actions_std.pt");
					torch::save(normalized_actions, data_folder_ + "/normalized_actions.pt");

					torch::Tensor coverage_features_mean = at::mean(coverage_features_.view({-1,coverage_features_.size(2)}), 0);
					torch::Tensor coverage_features_std = at::std(coverage_features_.view({-1,coverage_features_.size(2)}), 0);
					torch::Tensor normalized_coverage_features = (coverage_features_ - coverage_features_mean)/coverage_features_std;
					torch::save(coverage_features_mean, data_folder_ + "/coverage_features_mean.pt");
					torch::save(coverage_features_std, data_folder_ + "/coverage_features_std.pt");
					torch::save(normalized_coverage_features, data_folder_ + "/normalized_coverage_features.pt");
				}
			}

			void PrintTensorSizes(std::ostream& os) {
				os << "Device type: " << device_type_ << std::endl;
				os << "actions_: shape: " << actions_.sizes() << std::endl;
				os <<  "actions: bytesize (MB): " << GetTensorByteSizeMB(actions_) << std::endl;
				os << "robot_positions_: " << robot_positions_.sizes() << std::endl;
				os << "robot_positions: bytesize (MB): " << GetTensorByteSizeMB(robot_positions_) << std::endl;
				os << "raw_local_maps_: " << raw_local_maps_.sizes() << std::endl;
				os << "raw_local_maps: bytesize (MB): " << GetTensorByteSizeMB(raw_local_maps_) << std::endl;
				os << "local_maps_: " << local_maps_.sizes() << std::endl;
				os << "local_maps_: bytesize (MB): " << GetTensorByteSizeMB(local_maps_) << std::endl;
				os << "obstacle_maps_: " << obstacle_maps_.sizes() << std::endl;
				os << "obstacle_maps_: bytesize (MB): " << GetTensorByteSizeMB(obstacle_maps_) << std::endl;
				os << "comm_maps_: " << comm_maps_.sizes() << std::endl;
				os << "comm_maps: bytesize (MB): " << GetTensorByteSizeMB(comm_maps_) << std::endl;
				os << "coverage_features_: " << coverage_features_.sizes() << std::endl;
				os << "coverage_features: bytesize (MB): " << GetTensorByteSizeMB(coverage_features_) << std::endl;
			}

			void LoadConfigs(std::string const &config_file) {
				std::cout << "Using config file: " << config_file << std::endl;
				// Check if config_file exists
				if(not std::filesystem::exists(config_file)) {
					throw std::runtime_error("Could not open config file: " + config_file);
				}

				config_ = toml::parse_file(config_file);
				auto toml_DataDir = config_["DataDir"].value<std::string>();
				if(toml_DataDir) {
					data_dir_ = toml_DataDir.value();
				} else {
					throw std::runtime_error("Could not find DataDir in config file: " + config_file);
				}
				data_folder_ = data_dir_ + "/data/" + data_dir_append_ + "/";

				// Check if config_["DataDir"] directory exists
				if(not std::filesystem::exists(data_dir_)) {
					throw std::runtime_error("Could not find data directory: " + data_dir_);
				}

				if(!std::filesystem::exists(data_folder_)) {
					std::filesystem::create_directories(data_folder_);
				}
				// Check if config_["EnvironmentConfig"] file exists
				auto toml_EnvConfig = config_["EnvironmentConfig"].value<std::string>();
				if(not toml_EnvConfig) {
					throw std::runtime_error("Could not find EnvironmentConfig in config file: " + config_file);
				}
				std::string env_config_file = data_dir_ + "/" + toml_EnvConfig.value();
				if(not std::filesystem::exists(env_config_file)) {
					throw std::runtime_error("Could not find environment config file: " + env_config_file);
				}
				env_params_ = CoverageControl::Parameters(env_config_file);

				auto toml_ResizerModel = config_["TorchVisionTransformJIT"].value<std::string>();
				if(not toml_ResizerModel) {
					throw std::runtime_error("Could not find TorchVisionTransformJIT in config file: " + config_file);
				}
				std::string resizer_model_path = data_dir_ + "/" + toml_ResizerModel.value();

				// Check if config_["ResizerModel"] file exists
				if(not std::filesystem::exists(resizer_model_path)) {
					throw std::runtime_error("Could not find resizer model file: " + resizer_model_path);
				}
				torchvision_resizer_ = torch::jit::load(resizer_model_path);
			}

	};

} // namespace CoverageControlTorch

#endif // COVERAGECONTROLTORCH_GENERATE_DATASET_H_

