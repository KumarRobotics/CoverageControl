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

struct Polygons_Cuda {
	float *x;
	float *y;
	int *sz;
	int num_pts;
	int num_polygons;
};

void generate_world_map_cuda(BND_Cuda *, Polygons_Cuda const &, int const, int const, float const, float const, float const, float *importance_vec, float &);

#endif /* COVERAGECONTROL_GENERATE_WORLD_MAP_H_ */

