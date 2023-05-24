/**
 * A function for generating dataset for coverage control
 *
 **/

#ifndef COVERAGECONTROLTORCH_GENERATE_DATASET_H_
#define COVERAGECONTROLTORCH_GENERATE_DATASET_H_

#include <memory>
#include <yaml-cpp/yaml.h>
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
			double converged_data_ratio_ = 0.0;
			size_t trigger_size_ = std::numeric_limits<size_t>::max();
			size_t trigger_count_ = 0;

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

				LoadConfigs(config_file);
				dataset_size_ = config_["pNumDataset"].as<size_t>();
				num_robots_ = env_params_.pNumRobots;
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
				PrintTensorSizes();

				Run();
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
							++dataset_count_;
							if(dataset_count_ % 100 == 0) {
								std::cout << dataset_count_ << " " << env_count_ << " " << num_steps << std::endl;
							}
						} else {
							converged = StepWithoutSave();
						}
						++num_steps;
					}
					size_t num_converged_data = std::ceil(converged_data_ratio_ * num_steps / every_num_step_);
					size_t converted_data_count = 0;
					while(converted_data_count < num_converged_data and dataset_count_ < dataset_size_) {
						converged = StepWithSave();
						++num_steps;
						++converted_data_count;
						++dataset_count_;
					}
				}
				ProcessCommunicationMaps();
				/* ProcessEdgeWeights(); */
				PrintTensorSizes();
			}

		private:
			inline auto GetTensorByteSize(torch::Tensor const &tensor) {
				return tensor.numel() * torch::elementSize(torch::typeMetaToScalarType(tensor.dtype()));
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
				actions_[dataset_count_] = torch::from_blob(actions.data(), {num_robots_, 2});
				robot_positions_[dataset_count_] = PointsVectorToTensor(env_->GetRobotPositions());
				coverage_features_[dataset_count_] = torch::from_blob(env_->GetLocalVoronoiFeatures().data(), {num_robots_, 7});
				local_maps_[trigger_count_] = env_->GetAllRobotsLocalMaps().clone();
				++trigger_count_;
				if(trigger_count_ == trigger_size_) {
					trigger_count_ = 0;
					ProcessLocalMaps();
				}
				auto error_flag = env_->StepActions(actions);
				// Print error and convergence flags
				return converged or error_flag;
			}

			void ProcessEdgeWeights() {
				double comm_range = env_params_.pCommunicationRange;
				auto onebyexp = expf(-1);
				robot_positions_.to(device_);
				auto  pairwise_dist_matrices= torch::cdist(robot_positions_, robot_positions_, 2);

				auto edge_weights = torch::exp(-(pairwise_dist_matrices.square())/(comm_range*comm_range)).to(torch::kCPU);
				F::threshold(edge_weights, F::ThresholdFuncOptions(onebyexp, 0).inplace(true));
			}


			void ProcessCommunicationMaps() {
				std::cout << "Processing communication maps" << std::endl;
				double comm_range = env_params_.pCommunicationRange;
				auto onebyexp = expf(-1);
				robot_positions_.to(torch::kCPU);

				auto comm_maps = maps_[1];

				for (T_idx_t b_idx = 0; b_idx < comm_maps.size(0); ++b_idx) {
					if(b_idx % 100 == 0) {
						std::cout << "Processing batch " << b_idx << std::endl;
					}
					auto scaled_relative_pos = torch::round((robot_positions_[b_idx].unsqueeze(1) - robot_positions_[b_idx].unsqueeze(0)) * map_size_/(comm_range * env_params_.pResolution * 2.) + (map_size_/2. - env_params_.pResolution/2.)).to(torch::kInt64);
					/* std::cout << "Processing batch " << b_idx << std::endl; */
					auto  pairwise_dist_matrices = torch::cdist(robot_positions_[b_idx], robot_positions_[b_idx], 2);
					for (T_idx_t r_idx = 0; r_idx < comm_maps.size(1); ++r_idx) {
						auto indices = scaled_relative_pos.index({r_idx, pairwise_dist_matrices[r_idx] < (comm_range - 0.001), Slice()});
						/* std::cout << scaled_relative_pos[r_idx] << std::endl; */
						/* std::cout << indices << std::endl; */
						/* std::cout << robot_positions_[b_idx] << std::endl; */
						comm_maps.index_put_({b_idx, r_idx, indices}, 1);
					}
				}
				comm_maps.to(device_);
				std::cout << "Computed communication maps" << std::endl;
			}

			void ProcessLocalMaps() {
				std::cout << "Processing local maps" << std::endl;
				// Resize local maps
				size_t trigger_start_idx = trigger_count_ * trigger_size_;
				size_t trigger_end_idx = trigger_start_idx + trigger_size_ - 1;

				if(trigger_start_idx > dataset_size_ - 1) { return; }
				if(trigger_end_idx > dataset_size_ -1) {
					trigger_end_idx = dataset_size_ - 1;
				}
				local_maps_.to(device_);
				auto local_maps = local_maps_;
				local_maps.view({-1, map_size_, map_size_});
				auto output = torchvision_resizer_.forward({local_maps}).toTensor().to(torch::kCPU);
				std::cout << "output: " << output.sizes() << std::endl;
				auto transformed_local_maps = output.view({-1, num_robots_, map_size_, map_size_});
				maps_[0].index_put_({Slice(trigger_start_idx, trigger_end_idx + 1)}, transformed_local_maps);
				trigger_start_idx = trigger_end_idx;
				std::cout << "Local maps shape: " << local_maps_.sizes() << std::endl;
				local_maps_.to(torch::kCPU);
			}

			void PrintTensorSizes() {
				std::cout << "Device type: " << device_type_ << std::endl;
				std::cout << "actions_: shape: " << actions_.sizes() << std::endl;
				std::cout <<  "actions: bytesize (MB): " << actions_.numel() * torch::elementSize(torch::typeMetaToScalarType(actions_.dtype())) / 1000000 << std::endl;
				std::cout << "robot_positions_: " << robot_positions_.sizes() << std::endl;
				std::cout << "robot_positions: bytesize (MB): " << robot_positions_.numel() * torch::elementSize(torch::typeMetaToScalarType(robot_positions_.dtype())) / 1000000 << std::endl;
				std::cout << "local_maps_: " << local_maps_.sizes() << std::endl;
				std::cout << "local_maps: bytesize (MB): " << local_maps_.numel() * torch::elementSize(torch::typeMetaToScalarType(local_maps_.dtype())) / 1000000 << std::endl;
				std::cout << "maps_: " << maps_.sizes() << std::endl;
				std::cout << "maps: bytesize (MB): " << maps_.numel() * torch::elementSize(torch::typeMetaToScalarType(maps_.dtype())) / 1000000 << std::endl;
				std::cout << "coverage_features_: " << coverage_features_.sizes() << std::endl;
				std::cout << "coverage_features: bytesize (MB): " << coverage_features_.numel() * torch::elementSize(torch::typeMetaToScalarType(coverage_features_.dtype())) / 1000000 << std::endl;
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

