/*!
 * This file is part of the CoverageControl library
 * The file contains CUDA function declarations for generating world map
 *
 * TODO:
 *
 * @author Saurav Agarwal
 * @contact sauravag@seas.upenn.edu, agr.saurav1@gmail.com
 * Repository: https://github.com/KumarRobotics/CoverageControl
 *
 * The CoverageControl library is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
 *
 * DISCLAIMER OF WARRANTIES: THE SOFTWARE IS PROVIDED "AS-IS" WITHOUT WARRANTY OF ANY KIND INCLUDING ANY WARRANTIES OF PERFORMANCE OR MERCHANTABILITY OR FITNESS FOR A PARTICULAR USE OR PURPOSE OR OF NON-INFRINGEMENT. YOU BEAR ALL RISK RELATING TO QUALITY AND PERFORMANCE OF THE SOFTWARE OR HARDWARE.
 *
 * SUPPORT AND MAINTENANCE: No support, installation, or training is provided.
 *
 * You should have received a copy of the GNU General Public License along with CoverageControl library. If not, see <https://www.gnu.org/licenses/>.
 */


#ifndef COVERAGECONTROL_GENERATE_WORLD_MAP_H_
#define COVERAGECONTROL_GENERATE_WORLD_MAP_H_

#include <sstream>
#include <iostream>
#include <vector>
#include <limits>

namespace CoverageControl {
	float const kFloatMax = std::numeric_limits<float>::max(); //!< Maximum value of float
	float const kFloatMin = std::numeric_limits<float>::min(); //!< Minimum value of float

	//! Structure to store the parameters of the Bivariate Normal Distribution
	struct BND_Cuda {
		float mean_x, mean_y;
		float sigma_x, sigma_y;
		float scale, rho;
	};

	//! Structure to store the rectangular bounds of the polygons
	struct Bounds { float xmin = kFloatMax; float xmax = kFloatMin; float ymin = kFloatMax; float ymax = kFloatMin; };

	//! Structure to store the parameters of the polygons on the host
	struct Polygons_Cuda_Host {
		std::vector <float> x;
		std::vector <float> y;
		std::vector <float> imp;
		std::vector <int> sz;
		std::vector <Bounds> bounds;
		int num_pts = 0;
		int num_polygons = 0;
	};

	//! Structure to store the parameters of the polygons on the device
	struct Polygons_Cuda {
		float *x = nullptr;
		float *y = nullptr;
		float *imp = nullptr;
		int *sz = nullptr;
		Bounds *bounds = nullptr;
		int num_pts = 0;
		int num_polygons = 0;
	};

	//! Function to generate the world map on the device
	void generate_world_map_cuda(BND_Cuda *, Polygons_Cuda_Host const &, int const, int const, float const, float const, float const, float *importance_vec, float &);
} /* namespace CoverageControl */

#endif /* COVERAGECONTROL_GENERATE_WORLD_MAP_H_ */

