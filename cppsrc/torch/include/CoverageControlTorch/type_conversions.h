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

#ifndef COVERAGECONTROLTORCH_TYPE_CONVERSION_H_
#define COVERAGECONTROLTORCH_TYPE_CONVERSION_H_

#include <vector>
#include <Eigen/Dense>
#include <torch/torch.h>

typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> EigenMat;
namespace CoverageControlTorch {

	std::vector<char> get_the_bytes(std::string const &filename) {
		std::ifstream input(filename, std::ios::binary);
		std::vector<char> bytes(
				(std::istreambuf_iterator<char>(input)),
				(std::istreambuf_iterator<char>()));

		input.close();
		return bytes;
	}

	torch::Tensor GetTensorFromBytes(std::string const &filename) {
		std::vector<char> bytes = get_the_bytes(filename);
		torch::IValue x = torch::pickle_load(bytes);
		torch::Tensor my_tensor = x.toTensor();
		return my_tensor;
	}

	torch::Tensor ToTensor(EigenMat const &M) {
		EigenMat M_copy = M;
		std::vector<int64_t> dims = {M_copy.rows(), M_copy.cols()};
		torch::Tensor T = torch::from_blob(M_copy.data(), dims).clone();
		return T;
	}

	torch::Tensor ToTensor(double const &data) {
		torch::Tensor T = torch::zeros(1);
		T[0] = (float)data;
		return T;
	}

	torch::Tensor ToTensor(Eigen::Vector2d const &p) {
		torch::Tensor T = torch::zeros(2);
		T[0] = (float)p[0];
		T[1] = (float)p[1];
		return T;
	}

	torch::Tensor ToTensor(std::vector<double> const &vec) {
		torch::Tensor T = torch::zeros(vec.size());
		for (int i = 0; i < vec.size(); ++i) {
			T[i] = (float)vec[i];
		}
		return T;
	}

	torch::Tensor ToTensor(std::vector<std::vector<double>> const &vec) {
		std::vector<int64_t> dims = {vec.size(), vec[0].size()};
		torch::Tensor T = torch::zeros(dims);
		for (int i = 0; i < vec.size(); ++i) {
			T[i] = ToTensor(vec[i]);
		}
		return T;
	}

	torch::Tensor ToTensor(std::vector<Eigen::Vector2d> const &vec) {
		std::vector<int64_t> dims = {vec.size(), 2};
		torch::Tensor T = torch::zeros(dims);
		for (int i = 0; i < vec.size(); ++i) {
			T[i] = ToTensor(vec[i]);
		}
		return T;
	}

} // namespace CoverageControlTorch

#endif // COVERAGECONTROLTORCH_TYPE_CONVERSION_H_
