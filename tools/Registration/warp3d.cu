
// Created by A. Aichert on Thu July 9th 2018
#include <iostream>

#include <LibUtilsCuda/cutil_math.h>
#include <LibUtilsCuda/CudaMemory.h>
#include <LibUtilsCuda/UtilsCuda.hxx>

#include <LibUtilsCuda/culaut/culaut.hxx>

///
/// Simple backward warping via inverse "from in to out".
///

__global__ void kernel_warp_3d1c(culaut::Array<float,16> T_inv, int n_x_in, int n_y_in, int n_z_in, cudaTextureObject_t input_image, int n_x_out, int n_y_out, int n_z_out, float* output_d)
{
	// Find index of current thread
	int idx_x = blockIdx.x * blockDim.x + threadIdx.x;
	int idx_y = blockIdx.y * blockDim.y + threadIdx.y;
	int idx_z = blockIdx.z * blockDim.z + threadIdx.z;
	if (idx_x>=n_x_out) return;
	if (idx_y>=n_y_out) return;
	if (idx_z>=n_z_out) return;
	int idx=idx_z*n_x_out*n_y_out+idx_y*n_x_out+idx_x;

	float x_out[]={(float)idx_x, (float)idx_y, (float)idx_z, 1.0f};
	float x_in[4];
	culaut::xgemm<float,4,4,4>(T_inv.v,x_out,x_in);
	culaut::xvdehomogenize<float,4>(x_in);
	output_d[idx]=tex3D<float>(input_image,x_in[0]+.5f,x_in[1]+.5f,x_in[2]+.5f);
}

void cuda_warp_image_3d1c(culaut::Array<float,16> T_inv, int n_x_in, int n_y_in, int n_z_in, cudaTextureObject_t input_image, int n_x_out, int n_y_out, int n_z_out, float* output_d)
{
	dim3 block_size;
	block_size.x=8;
	block_size.y=8;
	block_size.y=8;
	dim3 grid_size;
	grid_size.x = iDivUp(n_x_out,block_size.x);
	grid_size.y = iDivUp(n_y_out,block_size.y);
	grid_size.z = iDivUp(n_z_out,block_size.z);

	kernel_warp_3d1c<<<grid_size, block_size>>>(T_inv, n_x_in, n_y_in, n_z_in, input_image,  n_x_out, n_y_out, n_z_out, output_d);
	cudaDeviceSynchronize();
	cudaCheckState
}

///
/// Difference image based on kernel_warp_3d1c
///

__global__ void kernel_warp_3d1c_difference(culaut::Array<float,16> T_inv, int n_x_in, int n_y_in, int n_z_in, cudaTextureObject_t input_image, cudaTextureObject_t reference_image, int n_x_out, int n_y_out, int n_z_out, float* output_d)
{
	// Find index of current thread
	int idx_x = blockIdx.x * blockDim.x + threadIdx.x;
	int idx_y = blockIdx.y * blockDim.y + threadIdx.y;
	int idx_z = blockIdx.z * blockDim.z + threadIdx.z;
	if (idx_x>=n_x_out) return;
	if (idx_y>=n_y_out) return;
	if (idx_z>=n_z_out) return;
	int idx=idx_z*n_x_out*n_y_out+idx_y*n_x_out+idx_x;

	float x_out[]={(float)idx_x, (float)idx_y, (float)idx_z, 1.0f};
	float x_in[4];
	culaut::xgemm<float,4,4,4>(T_inv.v,x_out,x_in);
	culaut::xvdehomogenize<float,4>(x_in);
	float diff=(tex3D<float>(reference_image,x_out[0]+.5f,x_out[1]+.5f,x_out[2]+.5f)-tex3D<float>(input_image,x_in[0]+.5f,x_in[1]+.5f,x_in[2]+.5f))*30.f;
	output_d[idx]=diff*diff/(float)(n_x_in*n_y_in*n_z_in);
}

void cuda_warp_image_3d1c_difference(culaut::Array<float,16> T_inv, int n_x_in, int n_y_in, int n_z_in, cudaTextureObject_t input_image, cudaTextureObject_t reference_image, int n_x_out, int n_y_out, int n_z_out, float* output_d)
{
	dim3 block_size;
	block_size.x=8;
	block_size.y=8;
	block_size.y=8;
	dim3 grid_size;
	grid_size.x = iDivUp(n_x_out,block_size.x);
	grid_size.y = iDivUp(n_y_out,block_size.y);
	grid_size.z = iDivUp(n_z_out,block_size.z);

	kernel_warp_3d1c_difference<<<grid_size, block_size>>>(T_inv, n_x_in, n_y_in, n_z_in, input_image, reference_image, n_x_out, n_y_out, n_z_out, output_d);
	cudaDeviceSynchronize();
	cudaCheckState
}

// Fast CUDA reduction
#include <thrust/reduce.h>
#include <thrust/transform.h>
#include <thrust/transform_reduce.h>
#include <thrust/functional.h>
#include <thrust/device_ptr.h>
#include <thrust/execution_policy.h>


// template<typename T>
// struct identity : public unary_function<T,T> { __host__ __device__ T operator()(const T &x) const { return x; }};

float cuda_sum_reduction(const UtilsCuda::MemoryView<float> &data)
{
	thrust::device_ptr<float> cptr = thrust::device_pointer_cast((float*)data);
	return thrust::reduce(cptr, cptr + data.size());
}

