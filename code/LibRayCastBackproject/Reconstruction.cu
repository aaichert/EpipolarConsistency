
// Created by A. Aichert on Thu Aug 30th 2018
#include <iostream>

#include <LibUtilsCuda/UtilsCuda.hxx>

#include <LibUtilsCuda/CudaMemory.h>
#include <LibUtilsCuda/CudaBindlessTexture.h>

#include <LibUtilsCuda/culaut/culaut.hxx>


__global__ void kernel_backprojection(
	unsigned short n_x, unsigned short n_y, unsigned short n_z, float *voxel,
	culaut::Array<float,12> projection_matrix_vx_px, cudaTextureObject_t image)
{
	// Find index of current thread
	int idx_x = blockIdx.x * blockDim.x + threadIdx.x;
	int idx_y = blockIdx.y * blockDim.y + threadIdx.y;
	int idx_z = blockIdx.z * blockDim.z + threadIdx.z;
	if (idx_x>=n_x) return;
	if (idx_y>=n_y) return;
	if (idx_z>=n_z) return;
	int idx=idx_z*n_y*n_x+idx_y*n_x+idx_x;

	// Homogeneous point in voxels
	float X[] = {(float)idx_x, (float)idx_y, (float)idx_z, 1.0f};

	// Projection to image
	float x[3];
	culaut::xgemm<float,3,4,1>(projection_matrix_vx_px.v,X,x);
	culaut::xvdehomogenize<float,3>(x);

	// Sample the image
	float &u=x[0];
	float &v=x[1];
	float intensity=tex2D<float>(image,u,v);

	// Backproject in to current voxel
	voxel[idx]+=intensity;

}


void backprojection(int n_x, int n_y, int n_z, float *voxel_d, culaut::Array<float,12> projection_matrix_vx_px, cudaTextureObject_t image )
{
	dim3 block_size;
	block_size.x=8;
	block_size.y=8;
	block_size.z=8;
	dim3 grid_size;
	grid_size.x = iDivUp(n_x,block_size.x);
	grid_size.y = iDivUp(n_y,block_size.y);
	grid_size.z = iDivUp(n_z,block_size.z);

	kernel_backprojection<<<grid_size, block_size>>>( n_x, n_y, n_z, voxel_d, projection_matrix_vx_px, image);
	cudaDeviceSynchronize();
	cudaCheckState

}
