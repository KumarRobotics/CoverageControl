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

#include <torch/torch.h>
#include <iostream>
#include "../include/CoverageControlTorch/base.h"

namespace CoverageControlTorch {
	int base() {
		torch::Tensor tensor = torch::rand({2, 3});
		/* std::cout << tensor << std::endl; */
		/* torch::Tensor gpu_tensor = tensor.to(torch::kCUDA); */
		/* std::cout << gpu_tensor << std::endl; */
		/* Parameters params; */
		/* WorldIDF world_idf(params); */
		/* CoverageSystem system(params, 2, 2); */
		return 0;
	}
} /* namespace CoverageControlTorch */
