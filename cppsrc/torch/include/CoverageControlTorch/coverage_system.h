/**
 * Extends the CoverageControl coverage system to include a torch map
 *
 **/

#ifndef COVERAGECONTROLTORCH_COVERAGE_SYSTEM_H_
#define COVERAGECONTROLTORCH_COVERAGE_SYSTEM_H_

#include <vector>
#include <CoverageControl/coverage_system.h>
#include <torch/torch.h>
#include <torch/script.h>

#include "type_conversions.h"

typedef long int T_idx_t;
using namespace torch::indexing;
using namespace CoverageControl;
namespace CoverageControlTorch {

	class CoverageSystem : public CoverageControl::CoverageSystem {
		private:
			float env_resolution_ = 1;
			float comm_range_ = 256;

			void init() {
				env_resolution_ = (float) params_.pResolution;
				comm_range_ = (float) params_.pCommunicationRange;
			}

		public:

			CoverageSystem(Parameters const &params, size_t const num_features, size_t const num_robots) : CoverageControl::CoverageSystem(params, num_features, num_robots) { init(); }

			CoverageSystem(Parameters const &params, WorldIDF const &world_idf, std::string const &pos_file_name) : CoverageControl::CoverageSystem(params, world_idf, pos_file_name) { init(); }

			CoverageSystem(Parameters const &params, WorldIDF const &world_idf, std::vector <Point2> const &robot_positions) : CoverageControl::CoverageSystem(params, world_idf, robot_positions) { init(); }

			CoverageSystem(Parameters const &params, std::vector <BivariateNormalDistribution> const &dists, std::vector <Point2> const &robot_positions) : CoverageControl::CoverageSystem(params, dists, robot_positions) { init(); }

			torch::Tensor GetAllRobotsLocalMaps() {
				torch::Tensor maps = torch::zeros({num_robots_, params_.pLocalMapSize, params_.pLocalMapSize});
#pragma omp parallel for
				for(size_t i = 0; i < num_robots_; i++) {
					maps[i] = ToTensor(robots_[i].GetRobotLocalMap());
				}
				return maps;
			}

			void GetAllRobotsCommunicationMaps(torch::Tensor maps, size_t const &map_size) {
				float f_map_size = (float) map_size;
				torch::Tensor robot_positions = ToTensor(GetRobotPositions());
				torch::Tensor scaled_relative_pos = torch::round((robot_positions.unsqueeze(0) - robot_positions.unsqueeze(1)) * f_map_size/(comm_range_ * env_resolution_ * 2.) + (f_map_size/2. - env_resolution_/2.)).to(torch::kInt64);
				torch::Tensor pairwise_dist_matrices = torch::cdist(robot_positions, robot_positions, 2);
				torch::Tensor diagonal_mask = torch::eye(pairwise_dist_matrices.size(0)).to(torch::kBool);
				pairwise_dist_matrices.masked_fill_(diagonal_mask, comm_range_ + 1);
				for (T_idx_t r_idx = 0; r_idx < num_robots_; ++r_idx) {
					torch::Tensor indices = scaled_relative_pos.index({r_idx, pairwise_dist_matrices[r_idx] < (comm_range_ - 0.001), Slice()});
					maps.index_put_({r_idx, indices.index({Slice(),0}), indices.index({Slice(), 1})}, 1);
				}
			}
	};
} // namespace CoverageControlTorch

#endif // COVERAGECONTROLTORCH_COVERAGE_SYSTEM_H_

