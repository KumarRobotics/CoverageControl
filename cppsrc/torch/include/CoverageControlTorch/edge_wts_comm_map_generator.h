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

#ifndef COVERAGECONTROLTORCH_EDGE_WTS_COMM_MAP_GENERATOR_H_
#define COVERAGECONTROLTORCH_EDGE_WTS_COMM_MAP_GENERATOR_H_

#include <torch/torch.h>
#include <math.h>
namespace F = torch::nn::functional;
using namespace torch::indexing;
namespace CoverageControlTorch {
	// Define a new Module.
	struct EdgeWtsCommMapGenerator : torch::nn::Module {
		EdgeWtsCommMapGenerator (int size, double communication_range, double resolution) : size_(size), communication_range_(communication_range), resolution_(resolution) {
		}

		auto eval(torch::Tensor robot_positions) {
			torch::Tensor edge_weights;
			auto num_robots = robot_positions.size(-1);
			auto  pairwise_dist_matrices= torch::cdist(robot_positions, robot_positions, 2);
			/* auto neg_adjacency = pairwise_dist_matrices > communication_range_; */
			/* std::cout << "neg_adjacency: " << neg_adjacency.sizes() << std::endl; */
			/* std::cout << "neg_adjacency_type: " << neg_adjacency.dtype() << std::endl; */
			/* neg_adjacency.fill_diagonal_(true); */
			edge_weights = torch::exp(-(pairwise_dist_matrices.square())/(communication_range_*communication_range_));
			F::threshold(edge_weights, F::ThresholdFuncOptions(expf(-1), 0).inplace(true));
			/* edge_weights[neg_adjacency == true] = 0; */

			torch::Tensor comm_map = torch::empty({num_robots, size_, size_});
			auto relative_pos = robot_positions.unsqueeze(2) - robot_positions.unsqueeze(1);
			std::cout << "relative_pos: " << relative_pos.sizes() << std::endl;

			double comm_scale = (communication_range_ * 2.) / size_;
			torch::Tensor map_translation = torch::empty({2});
			map_translation.index_put_({0}, size_ * comm_scale * resolution_/2.);
			map_translation.index_put_({1}, size_ * comm_scale * resolution_/2.);
			for(int i = 0; i < num_robots; ++i) {
				for(int j = 0; j < num_robots; ++j) {
					if(i == j) { continue; }
					auto neighbor_pos = relative_pos.index({Slice(), i, j, Slice()}).to(torch::kCUDA);
					std::cout << "neighbor_pos: " << neighbor_pos.sizes() << std::endl;
					auto map_pos = neighbor_pos + map_translation;
					auto indices = torch::round(map_pos / (resolution_ * comm_scale));
					comm_map.index_put_({i, indices}, 1);
				}
			}
			std::vector <torch::Tensor> edge_wts_comm_map{edge_weights, comm_map};
			// Return edge weights and communication maps
			return edge_wts_comm_map;
		}

		int size_;
		double communication_range_, resolution_;
	};
}

/* auto neighbor_indices = (pairwise_dist_matrices[i] <= communication_range_).nonzero()[0]; */

/* auto neighbor_pos = relative_pos.index({Slice(), pairwise_dist_matrices.index({Slice(), i, Slice(), Slice()}) <= communication_range_}); */
/* auto neighbor_indices = (pairwise_dist_matrices[i] <= communication_range_).nonzero()[0]; */

/* auto neighbor_pos = relative_pos.index({Slice(), pairwise_dist_matrices.index({Slice(), i, Slice(), Slice()}) <= communication_range_}); */
/* double comm_scale = (communication_range_ * 2.) / size_; */
/* torch::Tensor map_translation = torch::empty({2}); */
/* map_translation.index_put_({0}, size_ * comm_scale * resolution_/2.); */
/* map_translation.index_put_({1}, size_ * comm_scale * resolution_/2.); */
/* auto map_pos = neighbor_pos + map_translation; */
/* auto indices = torch::round(map_pos / (resolution_ * comm_scale)); */
/* comm_map.index_put_({i, indices}, 1); */
#endif // COVERAGECONTROLTORCH_EDGE_WTS_COMM_MAP_GENERATOR_H_
