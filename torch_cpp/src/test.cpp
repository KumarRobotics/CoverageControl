#include <torch/torch.h>
#include <iostream>
#include <CoverageControl/coverage_system.h>

using namespace CoverageControl;
int main() {
  torch::Tensor tensor = torch::rand({2, 3});
  std::cout << tensor << std::endl;
	torch::Tensor gpu_tensor = tensor.to(torch::kCUDA);
	std::cout << gpu_tensor << std::endl;
	Parameters params;
	WorldIDF world_idf(params);
	CoverageSystem system(params, 2, 2);
	return 0;

}
