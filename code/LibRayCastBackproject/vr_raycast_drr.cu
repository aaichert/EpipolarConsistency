// Created by A. Aichert on Tue Oct 4th 2016
#include <iostream>

#include <LibUtilsCuda/UtilsCuda.hxx>

#include <LibUtilsCuda/CudaMemory.h>
#include <LibUtilsCuda/CudaBindlessTexture.h>

#include <LibUtilsCuda/culaut/culaut.hxx>

#include "vr_cuda.hxx"

template <int n_c>
__global__ void kernel_raycast_drr(
	unsigned short n_u, unsigned short n_v,
	float * pixel_data,				//< image data (RGBA)
	float * model_vx_to_mm,			//< volume model transform (voxels to world)
	cudaTextureObject_t voxel_data,	//< volume data
	float* ray_entry,				//< ray entry
	float* ray_exit,				//< ray exit
	float * noise,					//< noise for ray offsets
	float   samples_per_voxel		//< samples per voxel
	)
{
	// Find index of current thread
	int idx_x = blockIdx.x * blockDim.x + threadIdx.x;
	int idx_y = blockIdx.y * blockDim.y + threadIdx.y;
	if (idx_x>=n_u) return;
	if (idx_y>=n_v) return;
	int idx=idx_y*n_u+idx_x;

	// Access correct pixel
	float *entry=ray_entry+4*idx;
	float *exit =ray_exit+4*idx;
	float *pixel=pixel_data+n_c*idx;

	// ray_direction=exit-entry
	float ray_direction_vx[4];
	culaut::xvcpy<float,float,3>(ray_direction_vx,exit);
	culaut::xvsub<float,3>(ray_direction_vx,entry);
	ray_direction_vx[3]=0.0f;

	// Test if we have a valid ray and scale ray_direction to step size
	float ray_length_vx=culaut::xvnorm2<float,3>(ray_direction_vx);
	if (ray_length_vx<1.0f) {
		culaut::xvset<float,n_c>(pixel,0);
		return;
	}
	culaut::xvscale<float,3>(ray_direction_vx,1.0f/(ray_length_vx*samples_per_voxel));

	// Go to start position (with random offset)
	float voxel[4];
	culaut::xvlincomb<float,float,3>(voxel,entry,1.0f,ray_direction_vx,-noise[idx]);

	// Accumulate absorption coeffs
	float intensity=0;
	for (float a=0;a<ray_length_vx;a+=1.0/samples_per_voxel)
	{
		culaut::xvadd<float,3>(voxel,ray_direction_vx);
		float sample=tex3D<float>(voxel_data,voxel[0],voxel[1],voxel[2]);
		intensity+=sample;
	}

	// How large was one step in mm? Use "voxel" as temporary variable
	culaut::xgemv<float,4,4>(model_vx_to_mm,ray_direction_vx, voxel);
	float mm_per_sample=culaut::xvnorm2<float,3>(voxel);
	intensity*=mm_per_sample;

	culaut::xvset<float,n_c>(pixel,intensity);

	if (n_c==2) pixel[1]=ray_length_vx;
//	culaut::xvset4<float>(pixel,(float)idx_x,(float)idx_y,0.0f,1.f);
}

void raycast_drr(
	int n_u, int n_v, int n_c,		//< image size and number of channels
	float * pixel_data_d,			//< image data (RGBA)
	float * model_C_Pinv_d,			//< volume model transform (voxels to world)
	cudaTextureObject_t voxel_data,	//< volume data
	float* ray_entry_d,				//< ray entry
	float* ray_exit_d,				//< ray exit
	float * noise_d,				//< noise for ray offsets 
	float samples_per_voxel		//< samples per voxel
	)
{
	dim3 block_size;
	block_size.x=16;
	block_size.y=16;
	dim3 grid_size;
	grid_size.x = iDivUp(n_u,block_size.x);
	grid_size.y = iDivUp(n_v,block_size.y);

	if (n_c==1)
		kernel_raycast_drr<1><<<grid_size, block_size>>>(n_u, n_v, pixel_data_d, model_C_Pinv_d, voxel_data, ray_entry_d, ray_exit_d, noise_d, samples_per_voxel);
	else if (n_c==2) // special case for backprojection in recostuction procedures
		kernel_raycast_drr<2><<<grid_size, block_size>>>(n_u, n_v, pixel_data_d, model_C_Pinv_d, voxel_data, ray_entry_d, ray_exit_d, noise_d, samples_per_voxel);
	else if (n_c==3)
		kernel_raycast_drr<3><<<grid_size, block_size>>>(n_u, n_v, pixel_data_d, model_C_Pinv_d, voxel_data, ray_entry_d, ray_exit_d, noise_d, samples_per_voxel);
	else if (n_c==4)
		kernel_raycast_drr<4><<<grid_size, block_size>>>(n_u, n_v, pixel_data_d, model_C_Pinv_d, voxel_data, ray_entry_d, ray_exit_d, noise_d, samples_per_voxel);
	else std::cerr << __FILE__ << " : " << __LINE__ << " : invalid number of channels!" << std::endl;


	cudaDeviceSynchronize();
	cudaCheckState
}
