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

#ifndef COVERAGECONTROLTORCH_COVERAGE_SYSTEM_H_
#define COVERAGECONTROLTORCH_COVERAGE_SYSTEM_H_

#include <vector>
#include <CoverageControl/coverage_system.h>
#include <torch/script.h>
#include <torch/torch.h>

#include "CoverageControlTorch/cnn_backbone.h"
#include "CoverageControlTorch/gnn_backbone.h"
#include "CoverageControlTorch/mlp.h"
#include "type_conversions.h"

typedef long int T_idx_t;
using namespace torch::indexing;
namespace F = torch::nn::functional;
using namespace CoverageControl;
namespace CoverageControlTorch {

	class CoverageSystem : public CoverageControl::CoverageSystem {
		private:
			float env_resolution_ = 1;
			float comm_range_ = 256;
			int const cnn_map_size_ = 32;
			/* std::shared_ptr<CNNBackbone> cnn_backbone_; */
			CNNBackbone cnn_backbone_;
			GNNBackBone gnn_backbone_;
			MLP action_nn_;
			torch::jit::script::Module torchvision_resizer_;

			torch::Tensor sensor_information_;
			std::vector <std::vector<torch::Tensor>> gnn_parameters_weights_;
			std::vector <torch::Tensor> gnn_parameters_biases_;
			torch::Tensor actions_mean_;
			torch::Tensor actions_std_;
			torch::Tensor gnn_features_prev_;
			torch::Tensor gnn_features_curr_;


			std::vector<std::vector<int>> adj_mat_;
			std::vector<std::vector<int>> adj_mat_prev_;
			torch::Device device_ = torch::kCPU;

			int num_nodes = num_robots_;
			int nlayers = 2;
			int K = 3;
			int num_features = 34;
			int latent_size = 256;
			std::vector<torch::Tensor> X;
			std::vector<torch::Tensor> Y;
			std::vector<torch::Tensor> Z;

			PointVector robot_positions_prev;
			PointVector robot_positions_curr;
			std::vector <std::vector<torch::Tensor>> Y_prev;
			std::vector <std::vector<torch::Tensor>> Y_curr;
			std::vector <int> degrees_prev;
			std::vector <int> degrees_curr;
			torch::Tensor gnn_output;
			void init() {
				env_resolution_ = (float) params_.pResolution;
				comm_range_ = (float) params_.pCommunicationRange;
			}

		public:

			CoverageSystem(Parameters const &params, size_t const num_features, size_t const num_robots) : CoverageControl::CoverageSystem(params, num_features, num_robots) { init(); }

			CoverageSystem(Parameters const &params, WorldIDF const &world_idf, std::string const &pos_file_name) : CoverageControl::CoverageSystem(params, world_idf, pos_file_name) { init(); }

			CoverageSystem(Parameters const &params, WorldIDF const &world_idf, std::vector <Point2> const &robot_positions) : CoverageControl::CoverageSystem(params, world_idf, robot_positions) { init(); }

			CoverageSystem(Parameters const &params, std::vector <BivariateNormalDistribution> const &dists, std::vector <Point2> const &robot_positions) : CoverageControl::CoverageSystem(params, dists, robot_positions) { init(); }


			void InitializeGNNCNN(std::string const &base_dir) {
				degrees_prev = std::vector <int>(num_robots_, 0);
				degrees_curr = std::vector <int>(num_robots_, 0);

				std::string resizer_jit_script = base_dir + "/TorchVisionResize_32.pt";
				InitializeResizer(resizer_jit_script);
				std::string cnn_model_file = base_dir + "/cnn_backbone_cpp.pt";
				LoadCNNBackBone(cnn_model_file);
				std::string params_dir = base_dir + "/cpp/";
				LoadGNNParameters(params_dir);
				std::string gnn_model_file = base_dir + "/gnn_backbone_cpp.pt";
				LoadGNNBackBone(gnn_model_file);
				std::string action_nn_file = base_dir + "/action_nn_cpp.pt";
				LoadActionNN(action_nn_file);
				/* gnn_backbone_->LoadParameters(device_, params_dir); */
				/* gnn_backbone_->to(device_); */
				/* gnn_backbone_->eval(); */
				/* action_nn_->LoadParameters(params_dir); */
				/* action_nn_->to(device_); */
				/* action_nn_->eval(); */
				torch::load(actions_mean_, params_dir + "/actions_mean.pt");
				torch::load(actions_std_, params_dir + "/actions_std.pt");
				std::cout << "Loaded GNN parameters." << std::endl;
				for(int i = 0; i < num_robots_; ++i) {
					std::vector <torch::Tensor> Y_i;

					Y_i.push_back(torch::zeros({K + 1, num_features}));
					for(int l = 1; l < nlayers; ++l) {
						Y_i.push_back(torch::zeros({K + 1, latent_size}));
					}
					Y_prev.push_back(Y_i);
					Y_curr.push_back(Y_i);
				}
				robot_positions_prev = GetRobotPositions();
				robot_positions_curr = GetRobotPositions();
				gnn_output = torch::zeros({num_robots_, latent_size});
				for(int i = 0; i < num_robots_; ++i) {
					degrees_curr[i] = GetNeighboringRobots(i).size();
				}
				gnn_features_curr_ = torch::zeros({num_robots_, num_features});
				gnn_features_prev_ = torch::zeros({num_robots_, num_features});
				/* adj_mat_ = torch::zeros({num_robots_, num_robots_}); */
				/* adj_mat_prev_ = torch::zeros({num_robots_, num_robots_}); */
				adj_mat_ = std::vector<std::vector<int>> (num_robots_, std::vector<int> (num_robots_, 0));
				adj_mat_prev_ = std::vector<std::vector<int>> (num_robots_, std::vector<int> (num_robots_, 0));
			}

			void InitializeResizer(std::string const &resizer_jit_script) {
				torchvision_resizer_ = torch::jit::load(resizer_jit_script);
			}

			torch::Tensor Resize(torch::Tensor const &input) {
				torch::Tensor output = torchvision_resizer_.forward({input}).toTensor();
				return output;
			}

			void StepCNNGNNDist(int freq_ratio, bool mode_test = false) {
				UpdateSensorInformation();
				torch::Tensor cnn_input = sensor_information_;
				cnn_input = cnn_input.view({-1, cnn_input.size(-3), cnn_input.size(-2), cnn_input.size(-1)});
				torch::Tensor cnn_output = cnn_backbone_->forward(cnn_input);
				torch::Tensor robot_positions = ToTensor(GetRobotPositions());
				robot_positions = (robot_positions + params_.pWorldMapSize/2.) / params_.pWorldMapSize;
				/* torch::Tensor gnn_features = torch::cat({cnn_output, robot_positions}, 1); */
				gnn_features_curr_ = torch::cat({cnn_output, robot_positions}, 1);
				ComputeAdjacencyMatrix();
				/* std::cout << "input: " << gnn_features_curr_[0] << std::endl; */

				/* std::cout << gnn_features << std::endl; */
				// Print robot_positions_prev and degrees_prev
				/* std::cout << "====================" << std::endl; */
				/* for(int i = 0; i < num_robots_; ++i) { */
				/* 	std::cout << robot_positions_prev[i][0] << " " << robot_positions_prev[i][1]<< std::endl; */
				/* 	std::cout << degrees_prev[i] << std::endl; */
				/* } */
				/* std::cout << "====================" << std::endl; */

				/* std::cout << "====================" << std::endl; */
				/* std::cout << "Comment these out\n"; */
				/* gnn_features_prev_ = gnn_features_curr_.detach().clone(); */
				/* robot_positions_prev = GetRobotPositions(); */
				/* adj_mat_prev_ = adj_mat_; */
				/* std::cout << "====================" << std::endl; */
				for(int f = 0; f < freq_ratio; ++f) {
					/* Y_prev = Y_curr; */
					for(int i = 0; i < Y_prev.size(); ++i) {
						int jlen = Y_prev[i].size();
						for(int j = 0; j < jlen; ++j) {
						Y_prev[i][j] = Y_curr[i][j].detach().clone();
						}
					}
#pragma omp parallel for
					for(int i = 0; i < num_robots_; ++i) {
						/* float bar_rc = params_.pCommunicationRange - 2 * nlayers * K * params_.pMaxRobotSpeed * params_.pTimeStep/freq_ratio; */
						/* bar_rc = std::max(bar_rc, 0.0f); */
						/* bar_rc = params_.pCommunicationRange; */
						gnn_output[i] = GNNInferenceDist(gnn_features_prev_[i], i);
					}
				}


				torch::Tensor controls = action_nn_->forward(gnn_output);
				torch::Tensor actions_tensor = controls * actions_std_ + actions_mean_;
				PointVector actions;
				for(int i = 0; i < num_robots_; ++i) {
					Point2 action(actions_tensor[i][0].item<float>(), actions_tensor[i][1].item<float>());
					actions.push_back(action);
				}
				StepActions(actions);

				gnn_features_prev_ = gnn_features_curr_.detach().clone();
				/* degrees_prev = degrees_curr; */
				robot_positions_prev = GetRobotPositions();
				adj_mat_prev_ = adj_mat_;
			}

			void StepCNNGNN() {
				UpdateSensorInformation();
				torch::Tensor actions_tensor = GNNCNNInference();
				PointVector actions;
				for(int i = 0; i < num_robots_; ++i) {
					Point2 action(actions_tensor[i][0].item<float>(), actions_tensor[i][1].item<float>());
					actions.push_back(action);
				}
				StepActions(actions);
			}

			std::list <int> GetNeighboringRobots(int const &robot_idx) {
				std::list <int> neighbors;
				auto robot_pos = GetRobotPositions();
				for(int i = 0; i < num_robots_; ++i) {
					if(i == robot_idx) {
						continue;
					}
					float dist = (robot_pos[robot_idx] - robot_pos[i]).norm();
					if(dist < params_.pCommunicationRange) {
						neighbors.push_back(i);
					}
				}
				return neighbors;
			}

			void UpdateSensorInformation() {
				sensor_information_ = torch::zeros({num_robots_, 4, cnn_map_size_, cnn_map_size_});
				torch::Tensor local_maps = GetAllRobotsLocalMaps();
				torch::Tensor obstacle_maps = GetAllRobotsObstacleMaps();
				torch::Tensor comm_maps = GetAllRobotsCommunicationMaps(cnn_map_size_);
				torch::Tensor resized_local_maps = Resize(local_maps);
				torch::Tensor resized_obstacle_maps = Resize(obstacle_maps);

				sensor_information_.index_put_({Slice(), 0, Slice(), Slice()}, resized_local_maps);
				sensor_information_.index_put_({Slice(), 1, Slice(), Slice()}, comm_maps.index({Slice(), 0, Slice(), Slice()}));
				sensor_information_.index_put_({Slice(), 2, Slice(), Slice()}, comm_maps.index({Slice(), 1, Slice(), Slice()}));
				sensor_information_.index_put_({Slice(), 3, Slice(), Slice()}, resized_obstacle_maps);
			}

			torch::Tensor GetSensorInformation(int const &robot_idx) {
				return sensor_information_[robot_idx];
			}

			torch::Tensor GetAllRobotsLocalMaps() {
				torch::Tensor maps = torch::zeros({num_robots_, params_.pLocalMapSize, params_.pLocalMapSize});
#pragma omp parallel for
				for(size_t i = 0; i < num_robots_; i++) {
					maps[i] = ToTensor(robots_[i].GetRobotLocalMap());
				}
				return maps;
			}

			torch::Tensor GetAllRobotsObstacleMaps() {
				torch::Tensor maps = torch::zeros({num_robots_, params_.pLocalMapSize, params_.pLocalMapSize});
#pragma omp parallel for
				for(size_t i = 0; i < num_robots_; i++) {
					maps[i] = ToTensor(robots_[i].GetObstacleMap());
				}
				return maps;
			}


			torch::Tensor GetAllRobotsCommunicationMaps(int const &map_size) {
				float f_map_size = (float) map_size;

				torch::Tensor comm_maps = torch::zeros({num_robots_, 2, map_size, map_size});
				torch::Tensor robot_positions = ToTensor(GetRobotPositions());

				torch::Tensor relative_pos = robot_positions.unsqueeze(0) - robot_positions.unsqueeze(1);
				torch::Tensor scaled_relative_pos = torch::round(relative_pos * f_map_size/(comm_range_ * env_resolution_ * 2.) + (f_map_size/2. - env_resolution_/2.)).to(torch::kInt64);
				torch::Tensor relative_dist = relative_pos.norm({2}, 2);
				torch::Tensor diagonal_mask = torch::eye(relative_dist.size(0)).to(torch::kBool);
				relative_dist.masked_fill_(diagonal_mask, comm_range_ + 1);

				for (T_idx_t r_idx = 0; r_idx < num_robots_; ++r_idx) {
					torch::Tensor comm_range_mask = relative_dist[r_idx] < (comm_range_ - 1e-5);
					torch::Tensor scaled_indices = scaled_relative_pos.index({r_idx, comm_range_mask, Slice()});
					torch::Tensor indices = scaled_indices.transpose(1, 0);
					torch::Tensor values = relative_pos.index({r_idx, comm_range_mask, Slice()})/comm_range_;
					comm_maps[r_idx][0] = torch::sparse_coo_tensor(indices, values.index({Slice(), 0}), {map_size, map_size}, values.dtype()).to_dense();
					comm_maps[r_idx][1] = torch::sparse_coo_tensor(indices, values.index({Slice(), 1}), {map_size, map_size}, values.dtype()).to_dense();
				}
				return comm_maps;
			}

			torch::Tensor GetEdgeWeights() {
				float onebyexp = expf(-1);
				torch::Tensor robot_positions = ToTensor(GetRobotPositions());
				torch::Tensor  pairwise_dist_matrices= torch::cdist(robot_positions, robot_positions, 2);
				torch::Tensor edge_weights = torch::exp(-(pairwise_dist_matrices.square())/(comm_range_*comm_range_)).to(torch::kCPU);
				F::threshold(edge_weights, F::ThresholdFuncOptions(onebyexp, 0).inplace(true));
				torch::Tensor diagonal_mask = torch::eye(edge_weights.size(1)).repeat({edge_weights.size(0), 1, 1}).to(torch::kBool);
				edge_weights.masked_fill_(diagonal_mask, 0);
				return edge_weights;
			}

			torch::Tensor GetLocalVoronoiFeaturesTensor() {
				return ToTensor(GetLocalVoronoiFeatures());
			}

			torch::Tensor GetGNNFeatures(torch::Tensor input) {
				/* torch::Tensor output = torch::zeros({num_robots_, 32, 32, 32}); */
				torch::Tensor output = cnn_backbone_->forward(input);
				return output;
			}

			int LoadActionNN(std::string const &action_nn_file) {
				action_nn_ = MLP();
				torch::load(action_nn_, action_nn_file);
				action_nn_->to(torch::kCPU);
				torch::NoGradGuard no_grad;
				action_nn_->eval();
				return 0;
			}
			int LoadGNNBackBone(std::string const &gnn_model_file) {
				gnn_backbone_ = GNNBackBone();
				torch::load(gnn_backbone_, gnn_model_file);
				gnn_backbone_->to(torch::kCPU);
				torch::NoGradGuard no_grad;
				gnn_backbone_->eval();
				return 0;
			}

			int LoadCNNBackBone(std::string const &cnn_model_file) {
				cnn_backbone_ = CNNBackbone(4,3,32,3,32);
				torch::load(cnn_backbone_, cnn_model_file);
				cnn_backbone_->to(torch::kCPU);
				torch::NoGradGuard no_grad;
				cnn_backbone_->eval();
				return 0;
			}

			int LoadCNNBackBoneJIT(std::string const &cnn_jit_file) {
				CNNBackbone cnn_backbone_(4,3,32,3,32);
				const torch::OrderedDict<std::string, at::Tensor>& model_params = cnn_backbone_->named_parameters();
				try {
					// Deserialize the ScriptModule from a file using torch::jit::load().
					std::cout << "Loading CNN backbone from " << cnn_jit_file << std::endl;
					torch::jit::script::Module cnn = torch::jit::load(cnn_jit_file);
					std::vector<std::string> param_names;
					for (auto const& w : model_params) {
						param_names.push_back(w.key());
					}

					std::cout << "Copying parameters from " << cnn_jit_file << " to model." << std::endl;

					torch::NoGradGuard no_grad;
					for(auto const &params:cnn.named_parameters()) {
						std::cout << params.name << " " << params.value.sizes() << std::endl;
						if (std::find(param_names.begin(), param_names.end(), params.name) != param_names.end()){
							model_params.find(params.name)->copy_(params.value);
						} else {
							std::cout << params.name << " does not exist among model parameters." << std::endl;
						};
					}

				}
				catch (const c10::Error& e) {
					std::cerr << "error loading the model\n";
					std::cerr << e.what() << std::endl;
					return -1;
				}
				cnn_backbone_->eval();
				torch::load(cnn_backbone_, cnn_jit_file);
				/* torch::load(cnn_backbone_, model_params); */


				/* cnn_backbone_.GetModule().get()->to(torch::kCPU); */
				/* cnn_backbone_.GetModule().get()->eval(); */
				/* // Save the model to a file */
				torch::save(cnn_backbone_, "cnn_backbone_cpp.pt");
				std::cout << "Saved model to cnn_backbone_cpp.pt" << std::endl;
			}

			void LoadGNNParameters(std::string const &base_dir) {
				int num_layers = 2;
				int K = 3;
				for(int l = 0; l < num_layers; ++l) {
					std::string bname = "bias_" + std::to_string(l);
					std::string bfilename = base_dir + bname + ".pt";
					torch::Tensor btensor;
					torch::load(btensor, bfilename);
					btensor = btensor.to(device_);
					gnn_parameters_biases_.push_back(btensor);
					std::vector<torch::Tensor> weights;
					weights.clear();
					for(int k = 0; k < K + 1; ++k) {
						std::string name = "lin_" + std::to_string(l) + "_" + std::to_string(k);
						std::string filename = base_dir + name + ".pt";
						torch::Tensor tensor;
						torch::load(tensor, filename);
						tensor = tensor.to(device_);
						weights.push_back(tensor);
					}
					gnn_parameters_weights_.push_back(weights);
				}
			}

			void ComputeAdjacencyMatrix() {
				torch::Tensor robot_positions = ToTensor(GetRobotPositions());
				int num_nodes = num_robots_;
				/* adj_mat_ = torch::zeros({num_nodes, num_nodes}); */
				/* adj_mat_ = adj_mat_.to(device_); */
#pragma omp parallel for
				for(int i = 0; i < num_nodes; ++i) {
					for(int j = 0; j < i; ++j) {
						float dist = (robot_positions[i] - robot_positions[j]).norm().item<float>();
						if(dist < params_.pCommunicationRange) {
							adj_mat_[i][j] = 1;
							adj_mat_[j][i] = 1;
						} else {
							adj_mat_[i][j] = 0;
							adj_mat_[j][i] = 0;
						}

					}
				}
			}

			torch::Tensor GNNInferenceDist(torch::Tensor input, int const &robot_idx) {
				torch::NoGradGuard no_grad_guard;
				/* auto neighbors = GetNeighboringRobots(robot_idx); */
				/* auto iter_n = neighbors.begin(); */
				/* while (iter_n != neighbors.end()) { */
				/* 	int n = *iter_n; */
				/* 	if((robot_positions_prev[robot_idx] - robot_positions_prev[n]).norm() > bar_rc) { */
				/* 		iter_n = neighbors.erase(iter_n); */
				/* 	} else { */
				/* 		++iter_n; */
				/* 	} */
				/* } */
				/* degrees_curr[robot_idx] = neighbors.size(); */

				std::vector <torch::Tensor> Y_i;
				Y_i.push_back(torch::zeros({K+1, num_features}));
				for(int l = 1; l < nlayers; ++l) {
					Y_i.push_back(torch::zeros({K+1, latent_size}));
				}

				torch::Tensor x_i = input.detach().clone();
				for(int l = 0; l < nlayers; ++l) {
					Y_i[l].index_put_({0}, x_i);
					torch::Tensor z_i = gnn_backbone_->forward(x_i, l, 0);
					for(int k = 1; k < K + 1; ++k) {
						/* int deg_i = degrees_prev[robot_idx]; */
						/* torch::Tensor deg_i = torch::sum(adj_mat_prev_.index({robot_idx})); */
						int deg_i = 0;
						for(int j = 0; j < num_nodes; ++j) {
							deg_i += adj_mat_prev_[robot_idx][j];
						}
						for(int j = 0; j < num_nodes; ++j) {
							if(robot_idx == j) {
								continue;
							}
								if(adj_mat_prev_[robot_idx][j] == 0) {
									continue;
								}
							/* if(adj_mat_prev_.index({robot_idx, j}).item<float>() == 0) { */
							/* 	continue; */
							/* } */
							/* for(auto j : neighbors)  */
							/* 	int deg_j = degrees_prev[j]; */
							/* torch::Tensor deg_j = torch::sum(adj_mat_prev_.index({j})); */
							int deg_j = 0;
							for(int jj = 0; jj < num_nodes; ++jj) {
								deg_j += adj_mat_prev_[j][jj];
							}
							Y_i[l].index_put_({k}, Y_i[l].index({k}) + Y_prev[j][l].index({k-1}) / std::sqrt(deg_i * deg_j));
						}
						z_i += gnn_backbone_->forward(Y_i[l].index({k}), l, k);
					}
					x_i = torch::relu(z_i);
				}
				Y_curr[robot_idx] = Y_i;
				return x_i;

			}

			torch::Tensor GNNInference(torch::Tensor const &input) {
				/* if(input.device() != device_) { */
				/* 	input.to(device_); */
				/* } */

				X.clear();
				Y.clear();
				Z.clear();
				for(int l = 0; l < nlayers + 1; ++l) {
					if(l == 0) {
						X.push_back(torch::zeros({num_nodes, num_features}));
						Y.push_back(torch::zeros({num_nodes, K + 1, num_features}));
						Z.push_back(torch::zeros({num_nodes, num_features}));
						continue;
					}
					if(l == 1) {
						X.push_back(torch::zeros({num_nodes, latent_size}));
						Y.push_back(torch::zeros({num_nodes, K + 1, num_features}));
						Z.push_back(torch::zeros({num_nodes, latent_size}));
						continue;
					}
					X.push_back(torch::zeros({num_nodes, latent_size}));
					Y.push_back(torch::zeros({num_nodes, K + 1, latent_size}));
					Z.push_back(torch::zeros({num_nodes, latent_size}));
				}

				X[0] = input;
				Z[0] = input;
				Y[0].index_put_({Slice(), 0, Slice()}, input);

				// For fully decentralized
				/* for(int i = 0; i < num_nodes; ++i) { */
				/* 	X[0].index_put_({i}, input.index({i})); */
				/* 	Z[0].index_put_({i}, input.index({i})); */
				/* 	Y[0].index_put_({i, 0}, input.index({i})); */
				/* } */

				for(int l = 1; l < nlayers + 1; ++l) {
					/* std::cout << "layer: " << l << std::endl; */
					for(int k = 0; k < K + 1; ++k) {
						/* std::cout << "k: " << k << std::endl; */
						for(int i = 0; i < num_nodes; ++i) {
							if(k == 0) {
								Y[l].index_put_({i, 0}, X[l-1].index({i}));
								/* Z[l].index_put_({i}, torch::matmul(Y[l].index({i, 0}), gnn_parameters_weights_[l-1][0].t())); */
								Z[l].index_put_({i}, gnn_backbone_->forward(Y[l].index({i, 0}), l - 1, k));
								continue;
							}
							/* torch::Tensor deg_i = torch::sum(adj_mat_.index({i})); */
							int deg_i = 0;
							for(int j = 0; j < num_nodes; ++j) {
								deg_i += adj_mat_[i][j];
							}
								
							for(int j = 0; j < num_nodes; ++j) {
								if(i == j) {
									continue;
								}
								if(adj_mat_[i][j] == 0) {
									continue;
								}
								/* if(adj_mat_.index({i, j}).item<float>() == 0) { */
								/* 	continue; */
								/* } */
								/* torch::Tensor deg_j = torch::sum(adj_mat_.index({j})); */
								int deg_j = 0;
								for(int jj = 0; jj < num_nodes; ++jj) {
									deg_j += adj_mat_[j][jj];
								}
								Y[l].index_put_({i, k}, Y[l].index({i, k}) + Y[l].index({j, k-1}) / std::sqrt(deg_i * deg_j));
							}
							/* Z[l].index_put_({i}, Z[l].index({i}) + torch::matmul(Y[l].index({i, k}), gnn_parameters_weights_[l-1][k].t())); */
							Z[l].index_put_({i}, Z[l].index({i}) + gnn_backbone_->forward(Y[l].index({i, k}), l - 1, k));
						}
					}
					/* Z[l] = Z[l] + gnn_parameters_biases_[l-1]; */
					X[l] = torch::relu(Z[l]);
					/* for(int i = 0; i < num_nodes; ++i) { */
					/* 	Z[l].index_put_({i}, Z[l].index({i}) + gnn_parameters_biases_[l-1]); */
					/* 	X[l].index_put_({i}, torch::relu(Z[l].index({i}))); */
					/* } */
				}
				return X[nlayers];
			}

			torch::Tensor GNNCNNInference() {
				/* torch::NoGradGuard no_grad; */
				/* std::cout << "GNNCNNInference" << std::endl; */
				ComputeAdjacencyMatrix();
				torch::Tensor cnn_input = sensor_information_;
				cnn_input = cnn_input.view({-1, cnn_input.size(-3), cnn_input.size(-2), cnn_input.size(-1)});
				torch::Tensor cnn_output = cnn_backbone_->forward(cnn_input);
				/* std::cout << cnn_output << std::endl; */
				/* std::cout << "CNN output: " << cnn_output.sizes() << std::endl; */
				/* cnn_output = cnn_output.view({num_robots_, 32}); */
				torch::Tensor robot_positions = ToTensor(GetRobotPositions());
				robot_positions = (robot_positions + params_.pWorldMapSize/2.) / params_.pWorldMapSize;
				/* std::cout << "Robot positions: " << robot_positions << std::endl; */
				// Concat robot positions with cnn output
				torch::Tensor gnn_input = torch::cat({cnn_output, robot_positions}, 1);
				/* std::cout << "GNN input: " << gnn_input.sizes() << std::endl; */
				torch::Tensor gnn_output = GNNInference(gnn_input);
				/* std::cout << gnn_output[0] << std::endl; */
				torch::Tensor controls = action_nn_->forward(gnn_output);
				/* std::cout << controls << std::endl; */
				controls = controls * actions_std_ + actions_mean_;
				return controls;
			}
	};
} // namespace CoverageControlTorch

#endif // COVERAGECONTROLTORCH_COVERAGE_SYSTEM_H_
