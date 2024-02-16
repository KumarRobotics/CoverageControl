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

#include <iostream>
#include <CoverageControlTorch/train_gnn.h>

int main(int argc, char* argv[]) {

	if (argc < 2) {
		std::cout << "Usage: ./train_gnn <yaml>" << std::endl;
		return 1;
	}

	std::string config_file = std::string(argv[1]);
	CoverageControlTorch::TrainGNN train_gnn(config_file);
	train_gnn.Train();

	return 0;
}
