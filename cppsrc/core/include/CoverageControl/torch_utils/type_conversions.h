/**
 * Type conversion between CoverageControl and Torch
 *
 * TODO: 
 **/

#ifndef COVERAGECONTROLTORCH_TYPE_CONVERSION_H_
#define COVERAGECONTROLTORCH_TYPE_CONVERSION_H_

#include <vector>
#include <Eigen/Dense>
#include <torch/torch.h>

typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> EigenMat;
namespace CoverageControl {

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
			for (size_t i = 0; i < vec.size(); ++i) {
				T[i] = (float)vec[i];
			}
			return T;
		}

		torch::Tensor ToTensor(std::vector<std::vector<double>> const &vec) {
			int64_t vec_size = int64_t(vec.size());
			int64_t feature_size = int64_t(vec[0].size());
			std::vector<int64_t> dims = {vec_size, feature_size};
			torch::Tensor T = torch::zeros(dims);
			for (size_t i = 0; i < vec.size(); ++i) {
				T[i] = ToTensor(vec[i]);
			}
			return T;
		}

		torch::Tensor ToTensor(std::vector<Eigen::Vector2d> const &vec) {
			int64_t vec_size = int64_t(vec.size());
			std::vector<int64_t> dims = {vec_size, 2};
			torch::Tensor T = torch::zeros(dims);
			for (size_t i = 0; i < vec.size(); ++i) {
				T[i] = ToTensor(vec[i]);
			}
			return T;
		}

		EigenMat ToEigenMat(torch::Tensor const &T) {
			float* data = T.data_ptr<float>();
			Eigen::Map<EigenMat> M(data, T.size(0), T.size(1));
			return M;
		}

} // namespace CoverageControlTorch

#endif // COVERAGECONTROLTORCH_TYPE_CONVERSION_H_
