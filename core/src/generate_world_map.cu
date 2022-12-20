/**
 * RTX 2080 Ti: CUDA Cores 4352, compute capablity: 7.5
 * 68 SMs, 64 CUDA Cores/SM
 * Block size: 32 x 32 = 1024 threads
 * mean_x, mean_y, sigma, scale, rho: 5 * # of distributions
 * resolution, size: 2
 * map: pWorldMapSize * pWorldMapSize
 **/

#include <CoverageControl/generate_world_map.ch>
#include <cuda_runtime.h>
#include <cmath>
#include <cuda_helpers/helper_cuda.h>
#include <thrust/extrema.h>
#include <thrust/device_ptr.h>

__device__ __constant__ int cu_num_dists;
__device__ __constant__ int cu_map_size;
__device__ __constant__ float cu_resolution;
__device__ __constant__ float cu_truncation;
__device__ __constant__ float cu_OneBySqrt2;
__device__ __constant__ float cu_normalization_factor;

__device__
float2 TransformPoint(BND_Cuda const *device_dists, int i, float2 const &in_point) {
	float2 pt;
	auto bnd = device_dists[i];
	if(bnd.rho == 0) {
		pt.x = (in_point.x - bnd.mean_x)/bnd.sigma_x;
		pt.y = (in_point.y - bnd.mean_y)/bnd.sigma_y;
		return pt;
	}
	pt.x = (in_point.x - bnd.mean_x)/bnd.sigma_x;
	pt.y = (in_point.y - bnd.mean_y)/bnd.sigma_y;
	pt.x = (pt.x - bnd.rho * pt.y)/(sqrt(1 - bnd.rho*bnd.rho));
	return pt;
}

__device__
float IntegrateQuarterPlane (BND_Cuda const &bnd, float2 const &in_point) {
	float2 pt;
	if(bnd.rho == 0) {
		pt.x = (in_point.x - bnd.mean_x)/bnd.sigma_x;
		pt.y = (in_point.y - bnd.mean_y)/bnd.sigma_y;
	} else {
		pt.x = (in_point.x - bnd.mean_x)/bnd.sigma_x;
		pt.y = (in_point.y - bnd.mean_y)/bnd.sigma_y;
		pt.x = (pt.x - bnd.rho * pt.y)/(sqrt(1 - bnd.rho*bnd.rho));
	}
	/* auto transformed_point = TransformPoint(i, in_point); */
	float sc = bnd.scale;
	/* return sc; */
	return sc * erfc(pt.x / sqrt(2.)) * erfc(pt.y / sqrt(2.))/4.;
}

__device__
float ComputeImportanceRectangle (BND_Cuda const *device_dists, float2 const &bottom_left, float2 const &top_right) {
	float2 bottom_right = make_float2(top_right.x, bottom_left.y);
	float2 top_left = make_float2(bottom_left.x, top_right.y);

	float total_importance = 0;
	for(int i = 0; i < cu_num_dists; ++i) {
		float2 mid_pt = make_float2((bottom_left.x + top_right.x)/2., (bottom_left.y + top_right.y)/2.);
		auto bnd = device_dists[i];
		if(bnd.rho == 0) {
			mid_pt.x = (mid_pt.x - bnd.mean_x)/bnd.sigma_x;
			mid_pt.y = (mid_pt.y - bnd.mean_y)/bnd.sigma_y;
		} else {
			mid_pt.x = (mid_pt.x - bnd.mean_x)/bnd.sigma_x;
			mid_pt.y = (mid_pt.y - bnd.mean_y)/bnd.sigma_y;
			mid_pt.x = (mid_pt.x - bnd.rho * mid_pt.y)/(sqrt(1 - bnd.rho*bnd.rho));
		}
		if(mid_pt.x * mid_pt.x + mid_pt.y * mid_pt.y > cu_truncation * cu_truncation + cu_resolution * cu_resolution) {
			/* printf("%f, %f, %f, %f\n", mid_pt.x, mid_pt.y, cu_truncation, cu_resolution); */
			continue;
		}
		total_importance += IntegrateQuarterPlane(bnd, bottom_left);
		total_importance -= IntegrateQuarterPlane(bnd, bottom_right);
		total_importance -= IntegrateQuarterPlane(bnd, top_left);
		total_importance += IntegrateQuarterPlane(bnd, top_right);
	}
	return total_importance;
}

__global__ void kernel (BND_Cuda const *device_dists, float *importance_vec) {
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	int idy = blockIdx.y * blockDim.y + threadIdx.y;
	int vec_idx = idx * cu_map_size + idy;
	if(not (idx < cu_map_size and idy < cu_map_size)) {
		return;
	}
	float2 bottom_left = make_float2(idx * cu_resolution, idy * cu_resolution);
	float2 top_right = make_float2(idx * cu_resolution + cu_resolution, idy * cu_resolution + cu_resolution);
	importance_vec[vec_idx] = ComputeImportanceRectangle(device_dists, bottom_left, top_right);
}

__global__ void normalize (float *importance_vec) {
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	int idy = blockIdx.y * blockDim.y + threadIdx.y;
	int vec_idx = idx * cu_map_size + idy;
	if(not (idx < cu_map_size and idy < cu_map_size)) {
		return;
	}
	importance_vec[vec_idx] *= cu_normalization_factor;
}
void generate_world_map_cuda(BND_Cuda *host_dists, Polygons_Cuda const &host_polygons, int const num_dists, int const map_size, float const resolution, float const truncation, float const pNorm, float *host_importance_vec, float &normalization_factor) {

	checkCudaErrors(cudaDeviceSynchronize());

	checkCudaErrors(cudaMemcpyToSymbol(cu_num_dists, &num_dists, sizeof(int)));
	checkCudaErrors(cudaMemcpyToSymbol(cu_map_size, &map_size, sizeof(int)));
	checkCudaErrors(cudaMemcpyToSymbol(cu_resolution, &resolution, sizeof(float)));
	checkCudaErrors(cudaMemcpyToSymbol(cu_truncation, &truncation, sizeof(float)));
	float f_OneBySqrt2 = (float)(1./std::sqrt(2.));
	checkCudaErrors(cudaMemcpyToSymbol(cu_OneBySqrt2, &f_OneBySqrt2, sizeof(float)));

	BND_Cuda *device_dists;
	checkCudaErrors(cudaMalloc(&device_dists, num_dists * sizeof(BND_Cuda)));
	/* checkCudaErrors(cudaMemcpyToSymbol(device_dists, &device_dists, sizeof(device_dists))); */
	checkCudaErrors(cudaMemcpy(device_dists, host_dists, num_dists * sizeof(BND_Cuda), cudaMemcpyHostToDevice));

	/* Polygons_Cuda device_polygons; */
	/* checkCudaErrors(cudaMalloc(&(device_polygons.x), host_polygons.num_pts * sizeof(float))); */
	/* checkCudaErrors(cudaMalloc(&(device_polygons.y), host_polygons.num_pts * sizeof(float))); */
	/* checkCudaErrors(cudaMalloc(&(device_polygons.sz), host_polygons.num_polygons * sizeof(int))); */
	/* device_polygons.num_pts = host_polygons.num_pts; */
	/* device_polygons.num_polygons = host_polygons.num_polygons; */

	float *device_importance_vec;
	checkCudaErrors(cudaMalloc(&device_importance_vec, map_size * map_size * sizeof(float)));


	dim3 dimBlock(16, 16, 1);
	dim3 dimGrid(map_size/dimBlock.x, map_size/dimBlock.x, 1);

	kernel <<<dimGrid, dimBlock>>>(device_dists, device_importance_vec);

	thrust::device_ptr<float> d_ptr = thrust::device_pointer_cast(device_importance_vec);
	float max = *(thrust::max_element(d_ptr, d_ptr + map_size * map_size));

	normalization_factor = pNorm;
	if(max > 1e-8) {
		normalization_factor = pNorm / max;
	}
	checkCudaErrors(cudaMemcpyToSymbol(cu_normalization_factor, &normalization_factor, sizeof(float)));
	normalize <<<dimGrid, dimBlock>>>(device_importance_vec);

	checkCudaErrors(cudaMemcpy(host_importance_vec, device_importance_vec, map_size * map_size * sizeof(float), cudaMemcpyDeviceToHost));

	checkCudaErrors(cudaFree(device_dists));
	checkCudaErrors(cudaFree(device_importance_vec));
	/* checkCudaErrors(cudaFree(device_polygons.x)); */
	/* checkCudaErrors(cudaFree(device_polygons.y)); */
	/* checkCudaErrors(cudaFree(device_polygons.sz)); */

	/* auto largest_val_ptr = thrust::max_element(host_importance_vec, host_importance_vec + map_size * map_size); */

	cudaError_t error = cudaGetLastError();
	if(error != cudaSuccess) {
		std::stringstream strstr;
		strstr << "run_kernel launch failed" << std::endl;
		throw strstr.str();
	}
}
