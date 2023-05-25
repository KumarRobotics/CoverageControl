/**
 * A function for generating dataset for coverage control
 *
 **/

#ifndef COVERAGECONTROLTORCH_GENERATE_DATASET_H_
#define COVERAGECONTROLTORCH_GENERATE_DATASET_H_

#include <iostream>
#include <fstream>
#include <memory>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <ctime> 
#include <CoverageControl/coverage_system.h>

#include <torch/script.h>
#include <torch/torch.h>
#include <CoverageControl/algorithms/lloyd_global_online.h>
#include <CoverageControl/algorithms/oracle_global_offline.h>


using namespace torch::indexing;
typedef long int T_idx_t;
namespace F = torch::nn::functional;
typedef CoverageControl::LloydGlobalOnline CoverageAlgorithm;
/* typedef CoverageControl::OracleGlobalOffline CoverageAlgorithm; */

#include "coverage_system.h"

namespace CoverageControlTorch {

	class GenerateDataset {

		private:
			YAML::Node config_;
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

			torch::DeviceType device_type_ = torch::kCPU;
			torch::Device device_ = device_type_;

			torch::Tensor actions_;
			torch::Tensor robot_positions_;
			torch::Tensor local_maps_;
			torch::Tensor maps_;
			torch::Tensor coverage_features_;

			torch::jit::script::Module torchvision_resizer_;

		public:
			GenerateDataset(std::string const &config_file) {
				auto start_time = std::chrono::system_clock::now();

				LoadConfigs(config_file);
				dataset_size_ = config_["pNumDataset"].as<size_t>();
				num_robots_ = env_params_.pNumRobots;
				comm_range_ = (float)env_params_.pCommunicationRange;
				env_resolution_ = (float)env_params_.pResolution;
				map_size_ = config_["pCNNMapSize"].as<int>();
				every_num_step_ = config_["pEveryNumSteps"].as<size_t>();
				trigger_size_ = config_["pTriggerPostProcessing"].as<size_t>();
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
				local_maps_ = torch::empty({trigger_size_, num_robots_, env_params_.pLocalMapSize, env_params_.pLocalMapSize});
				maps_ = torch::zeros({2, dataset_size_, num_robots_, map_size_, map_size_});
				coverage_features_ = torch::empty({dataset_size_, num_robots_, 7});
				PrintTensorSizes(std::cout);
				std::ofstream file;
				std::time_t start_time_t = std::chrono::system_clock::to_time_t(start_time);
				file.open(data_dir_ + "/init.txt");
				file << "Start time: " << std::ctime(&start_time_t) << std::endl;
				PrintTensorSizes(file);
				file.close();
				Run();
				auto end_time = std::chrono::system_clock::now();
				std::chrono::duration<double> elapsed_seconds = end_time - start_time;
				std::time_t end_time_t = std::chrono::system_clock::to_time_t(end_time);
				file.open(data_dir_ + "/init.txt", std::ios_base::app);
				file << "Finished computation at " << std::ctime(&end_time_t)
					<< "elapsed time: " << elapsed_seconds.count()/3600 << " hrs"
					<< std::endl;
				file.close();
			}

			void Run() {
				while(dataset_count_ < dataset_size_) {
					env_ = std::make_shared <CoverageSystem>(env_params_, env_params_.pNumFeatures, env_params_.pNumRobots);
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
					size_t num_converged_data = std::ceil(converged_data_ratio_ * num_steps / every_num_step_);
					size_t converged_data_count = 0;
					while(converged_data_count < num_converged_data and dataset_count_ < dataset_size_) {
						converged = StepWithSave();
						++converged_data_count;
					}
				}
				ProcessCommunicationMaps();
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
				/* local_maps_[trigger_count_] = env_->GetAllRobotsLocalMaps().clone(); */
				for(size_t i = 0; i < num_robots_; ++i) {
					local_maps_[trigger_count_][i] = ToTensor(env_->GetRobotLocalMap(i));
				}
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
				if(config_["pSaveAsSparseQ"].as<bool>()){
					torch::save(edge_weights.to_sparse(), data_dir_ + "edge_weights.pt");
				} else {
					torch::save(edge_weights, data_dir_ + "edge_weights.pt");
				}
			}


			void ProcessCommunicationMaps() {
				robot_positions_.to(torch::kCPU);
				torch::Tensor comm_maps = maps_[1];

				for (T_idx_t b_idx = 0; b_idx < comm_maps.size(0); ++b_idx) {
					torch::Tensor scaled_relative_pos = torch::round((robot_positions_[b_idx].unsqueeze(0) - robot_positions_[b_idx].unsqueeze(1)) * map_size_/(comm_range_ * env_resolution_ * 2.) + (map_size_/2. - env_resolution_/2.)).to(torch::kInt64);
					torch::Tensor pairwise_dist_matrices = torch::cdist(robot_positions_[b_idx], robot_positions_[b_idx], 2);
					torch::Tensor diagonal_mask = torch::eye(pairwise_dist_matrices.size(0)).to(torch::kBool);
					pairwise_dist_matrices.masked_fill_(diagonal_mask, comm_range_ + 1);

					for (T_idx_t r_idx = 0; r_idx < comm_maps.size(1); ++r_idx) {
						torch::Tensor indices = scaled_relative_pos.index({r_idx, pairwise_dist_matrices[r_idx] < (comm_range_ - 0.001), Slice()});
						comm_maps.index_put_({b_idx, r_idx, indices.index({Slice(),0}), indices.index({Slice(), 1})}, 1);
					}
				}
				/* comm_maps.to(device_); */
			}

			void ProcessLocalMaps() {
				std::cout << "Processing local maps" << std::endl;
				if(trigger_start_idx_ > dataset_size_ - 1) { return; }

				size_t trigger_end_idx = trigger_start_idx_ + trigger_size_;
				if(trigger_end_idx > dataset_size_) {
					trigger_end_idx = dataset_size_;
				}

				local_maps_.to(device_);
				torch::Tensor local_maps = local_maps_.view({-1, env_params_.pLocalMapSize, env_params_.pLocalMapSize});
				torch::Tensor output = torchvision_resizer_.forward({local_maps}).toTensor().to(torch::kCPU);
				torch::Tensor transformed_local_maps = output.view({-1, num_robots_, map_size_, map_size_});
				maps_[0].index_put_({Slice(trigger_start_idx_, trigger_end_idx)}, transformed_local_maps.clone());
				trigger_start_idx_ = trigger_end_idx;
				local_maps_.to(torch::kCPU);
			}

			void SaveDataset() {
				std::cout << "Saving dataset to " << data_dir_ << std::endl;
				// Create data subfolder if it does not exists
				std::string data_folder = data_dir_ + "/data/";
				if(!std::filesystem::exists(data_folder)) {
					std::filesystem::create_directory(data_folder);
				}

				torch::save(robot_positions_, data_folder + "/robot_positions.pt");
				torch::save(maps_[0].clone(), data_folder + "/local_maps.pt");
				if(config_["pSaveAsSparseQ"].as<bool>()) {
					torch::save(maps_[1].to_sparse(), data_folder + "/comm_maps.pt");
				}
				else {
					torch::save(maps_[1].clone(), data_folder + "/comm_maps.pt");
				}
				if(config_["pNormalizeQ"].as<bool>()) {
					torch::Tensor actions_mean = at::mean(actions_.view({-1,2}), 0);
					torch::Tensor actions_std = at::std(actions_.view({-1,2}), 0);
					torch::Tensor normalized_actions = (actions_ - actions_mean)/actions_std;
					torch::save(actions_mean, data_folder + "/actions_mean.pt");
					torch::save(actions_std, data_folder + "/actions_std.pt");
					torch::save(normalized_actions, data_folder + "/actions.pt");

					torch::Tensor coverage_features_mean = at::mean(coverage_features_.view({-1,coverage_features_.size(2)}), 0);
					torch::Tensor coverage_features_std = at::std(coverage_features_.view({-1,coverage_features_.size(2)}), 0);
					torch::Tensor normalized_coverage_features = (coverage_features_ - coverage_features_mean)/coverage_features_std;
					torch::save(coverage_features_mean, data_folder + "/coverage_features_mean.pt");
					torch::save(coverage_features_std, data_folder + "/coverage_features_std.pt");
					torch::save(normalized_coverage_features, data_folder + "/coverage_features.pt");
				} else {
					torch::save(actions_, data_folder + "/actions.pt");
					torch::save(coverage_features_, data_folder + "/coverage_features.pt");
				}
			}

			void PrintTensorSizes(std::ostream& os) {
				os << "Device type: " << device_type_ << std::endl;
				os << "actions_: shape: " << actions_.sizes() << std::endl;
				os <<  "actions: bytesize (MB): " << GetTensorByteSizeMB(actions_) << std::endl;
				os << "robot_positions_: " << robot_positions_.sizes() << std::endl;
				os << "robot_positions: bytesize (MB): " << GetTensorByteSizeMB(robot_positions_) << std::endl;
				os << "local_maps_: " << local_maps_.sizes() << std::endl;
				os << "local_maps: bytesize (MB): " << GetTensorByteSizeMB(local_maps_) << std::endl;
				os << "maps_: " << maps_.sizes() << std::endl;
				os << "maps: bytesize (MB): " << GetTensorByteSizeMB(maps_) << std::endl;
				os << "coverage_features_: " << coverage_features_.sizes() << std::endl;
				os << "coverage_features: bytesize (MB): " << GetTensorByteSizeMB(coverage_features_) << std::endl;
			}

			void LoadConfigs(std::string const &config_file) {
				std::cout << "Using config file: " << config_file << std::endl;
				// Check if config_file exists
				if(not std::filesystem::exists(config_file)) {
					throw std::runtime_error("Could not open config file: " + config_file);
				}

				config_ = YAML::LoadFile(config_file);
				data_dir_ = config_["pDataDir"].as<std::string>();

				// Check if config_["pDataDir"] directory exists
				std::string data_dir = config_["pDataDir"].as<std::string>();
				if(not std::filesystem::exists(data_dir)) {
					throw std::runtime_error("Could not find data directory: " + data_dir);
				}

				// Check if config_["pEnvironmentConfig"] file exists
				std::string env_config_file = data_dir + "/" + config_["pEnvironmentConfig"].as<std::string>();
				if(not std::filesystem::exists(env_config_file)) {
					throw std::runtime_error("Could not find environment config file: " + env_config_file);
				}
				env_params_ = CoverageControl::Parameters(env_config_file);

				std::string resizer_model_path = data_dir + "/" + config_["pTorchVisionTransformJIT"].as<std::string>();
				// Check if config_["pResizerModel"] file exists
				if(not std::filesystem::exists(resizer_model_path)) {
					throw std::runtime_error("Could not find resizer model file: " + resizer_model_path);
				}
				torchvision_resizer_ = torch::jit::load(resizer_model_path);
			}

	};

} // namespace CoverageControlTorch

#endif // COVERAGECONTROLTORCH_GENERATE_DATASET_H_

