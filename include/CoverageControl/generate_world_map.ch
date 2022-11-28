/**
 * Contains cuda function declarations for
 **/

#ifndef COVERAGECONTROL_GENERATE_WORLD_MAP_H_
#define COVERAGECONTROL_GENERATE_WORLD_MAP_H_

#include <sstream>
#include <iostream>
#include <vector>

struct BND_Cuda {
	float mean_x, mean_y;
	float sigma_x, sigma_y;
	float scale, rho;
};

void generate_world_map_cuda(BND_Cuda *host_dists, int num_dists, int map_size, float resolution, float truncation, float const, float *importance_vec, float &);

#endif /* COVERAGECONTROL_GENERATE_WORLD_MAP_H_ */

