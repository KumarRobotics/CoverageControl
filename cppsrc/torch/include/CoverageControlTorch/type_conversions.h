/**
 * Type conversion between CoverageControl and Torch
 *
 * TODO: 
 **/

#ifndef COVERAGECONTROLTORCH_TYPE_CONVERSION_H_
#define COVERAGECONTROLTORCH_TYPE_CONVERSION_H_

#include <vector>
#include <Eigen/Dense> // Eigen is used for maps

#include <CoverageControl/typedefs.h>

#include <torch/torch.h>

using namespace CoverageControl;
namespace CoverageControlTorch {

		torch::Tensor EigenToLibTorch(MapType const &M) {
			MapType M_copy = M;
			std::vector<int64_t> dims = {M_copy.rows(), M_copy.cols()};
			auto T = torch::from_blob(M_copy.data(), dims).clone();
			return T;
		}

} // namespace CoverageControlTorch

#endif // COVERAGECONTROLTORCH_TYPE_CONVERSION_H_
