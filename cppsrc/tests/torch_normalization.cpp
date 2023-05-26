#include <torch/torch.h>
#include <iostream>
#include <CoverageControlTorch/base.h>

using namespace CoverageControl;
int main() {
	base();

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
