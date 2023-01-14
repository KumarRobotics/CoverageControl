/**
 * Contains cuda function declarations for
 **/

#ifndef COVERAGECONTROL_GENERATE_WORLD_MAP_H_
#define COVERAGECONTROL_GENERATE_WORLD_MAP_H_

#include <sstream>
#include <iostream>
#include <vector>
#include <limits>

namespace CoverageControl {
float const kFloatMax = std::numeric_limits<float>::max();
float const kFloatMin = std::numeric_limits<float>::min();

struct BND_Cuda {
	float mean_x, mean_y;
	float sigma_x, sigma_y;
	float scale, rho;
};

struct Bounds { float xmin = kFloatMax; float xmax = kFloatMin; float ymin = kFloatMax; float ymax = kFloatMin; };

struct Polygons_Cuda_Host {
	std::vector <float> x;
	std::vector <float> y;
	std::vector <float> imp;
	std::vector <int> sz;
	std::vector <Bounds> bounds;
	int num_pts = 0;
	int num_polygons = 0;
};

struct Polygons_Cuda {
	float *x = nullptr;
	float *y = nullptr;
	float *imp = nullptr;
	int *sz = nullptr;
	Bounds *bounds = nullptr;
	int num_pts = 0;
	int num_polygons = 0;
};

void generate_world_map_cuda(BND_Cuda *, Polygons_Cuda_Host const &, int const, int const, float const, float const, float const, float *importance_vec, float &);
} /* namespace CoverageControl */

#endif /* COVERAGECONTROL_GENERATE_WORLD_MAP_H_ */

