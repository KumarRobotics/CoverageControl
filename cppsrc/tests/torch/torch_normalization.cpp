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

int main() {

	torch::Tensor a = torch::rand({3,3,2});
	print(a);
	std::cout << a << std::endl;

	torch::Tensor norm_a = a.norm({2},2);
	print(norm_a);
	std::cout << norm_a << std::endl;

	for(int i = 0; i < 3; ++i) {
		for(int j = 0; j < 3; ++j) {
			if(not torch::equal(a[i][j].norm(), norm_a[i][j])) {
				std::cout << "NOT EQUAL" << std::endl;
			}
		}
	}
}
