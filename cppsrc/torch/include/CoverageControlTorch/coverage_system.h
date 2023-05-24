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

using namespace torch::indexing;
namespace CoverageControlTorch {

	class CoverageSystem : public CoverageControl::CoverageSystem {

		public:

			CoverageSystem(Parameters const &params, size_t const num_features, size_t const num_robots) : CoverageControl::CoverageSystem(params, num_features, num_robots) { }

			CoverageSystem(Parameters const &params, WorldIDF const &world_idf, std::string const &pos_file_name) : CoverageControl::CoverageSystem(params, world_idf, pos_file_name) { }

			CoverageSystem(Parameters const &params, WorldIDF const &world_idf, std::vector <Point2> const &robot_positions) : CoverageControl::CoverageSystem(params, world_idf, robot_positions) { }

			CoverageSystem(Parameters const &params, std::vector <BivariateNormalDistribution> const &dists, std::vector <Point2> const &robot_positions) : CoverageControl::CoverageSystem(params, dists, robot_positions) { }

			torch::Tensor GetAllRobotsLocalMaps() {
				torch::Tensor maps = torch::zeros({num_robots_, params_.pLocalMapSize, params_.pLocalMapSize});
#pragma omp parallel for
				for(size_t i = 0; i < num_robots_; i++) {
					maps[i] = EigenToLibTorch(robots_[i].GetRobotLocalMap());
				}
				return maps;
			}

			void GetAllRobotsCommunicationMaps(torch::Tensor maps, size_t const &map_size) const {
#pragma omp parallel for
				for(int i = 0; i < num_robots_; i++) {
					auto robot_neighbors_pos = GetRobotsInCommunication(i);
					double comm_scale = (params_.pCommunicationRange * 2.) / map_size;
					Point2 map_translation(map_size * comm_scale * params_.pResolution/2., map_size * comm_scale * params_.pResolution/2.);
					for(Point2 const& relative_pos:robot_neighbors_pos) {
						Point2 map_pos = relative_pos + map_translation;
						int pos_idx, pos_idy;
						MapUtils::GetClosestGridCoordinate(params_.pResolution * comm_scale, map_pos, pos_idx, pos_idy);
						if(pos_idx < map_size and pos_idy < map_size and pos_idx >= 0 and pos_idy >= 0) {
							maps.index_put_({i, pos_idx, pos_idy}, 1);
						}
					}
				}
			}
	};
} // namespace CoverageControlTorch

#endif // COVERAGECONTROLTORCH_COVERAGE_SYSTEM_H_

