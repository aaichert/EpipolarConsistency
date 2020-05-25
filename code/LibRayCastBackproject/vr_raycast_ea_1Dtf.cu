// Created by A. Aichert on Tue Oct 4th 2016
#include <iostream>

#include <LibUtilsCuda/UtilsCuda.hxx>

#include <LibUtilsCuda/CudaMemory.h>
#include <LibUtilsCuda/CudaBindlessTexture.h>

#include <LibUtilsCuda/culaut/culaut.hxx>

#include "vr_cuda.hxx"

template <int n_c>
__global__ void kernel_raycast(
	unsigned short n_u, unsigned short n_v,
	float * pixel_data,				//< image data (RGBA)
	float * model_C_Pinv_h,			//< volume model transform (voxels to world)
	cudaTextureObject_t voxel_data,	//< volume data
	float * ray_entry,				//< ray entry
	float * ray_exit,				//< ray exit
	float * noise,					//< noise for ray offsets 
	float   samples_per_voxel,		//< samples per voxel
	cudaTextureObject_t tf			//< transfer function
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

	float ray_length=culaut::xvdistance2<float,3>(entry,exit);
	if (ray_length<=0)
	{
		culaut::xvset<float,n_c>(pixel,0.0f);
		return;
	}

	float step_in_mm=1;
	float4 intensity=make_float4(0,0,0,0);
	float step=1.0/ray_length;
	step/=samples_per_voxel;
	float voxel[3];
	for (float a=noise[idx]*step;a<1.0;a+=step)
	{
		culaut::xvlincomb<float,float,3>(voxel,entry,1.0-a,exit,a);
		float  sample=tex3D<float>(voxel_data,voxel[0],voxel[1],voxel[2]);
		float4 color=tex1D<float4>(tf,sample);
		color.w/=step_in_mm;
		color.x*=color.w;
		color.y*=color.w;
		color.z*=color.w;
		intensity.x+=color.x*(1.0-intensity.w);
		intensity.y+=color.y*(1.0-intensity.w);
		intensity.z+=color.z*(1.0-intensity.w);
		intensity.w+=color.w*(1.0-intensity.w);
		if (intensity.w>=1.0)
			break;
	}
	if (n_c>=4)
		culaut::xvcpy<float,float,4>(pixel,(float*)&intensity);
	else if (n_c>=3)
		culaut::xvcpy<float,float,3>(pixel,(float*)&intensity);
	else
		culaut::xvset<float,n_c>(pixel,culaut::xvnorm2<float,3>(&intensity.x));
}


void raycast_ea_1Dtf(
	int n_u, int n_v, int n_c,		//< image size and number of channels
	float * pixel_data_d,			//< image data (RGBA)
	float * model_C_Pinv_d,			//< volume model transform (voxels to world)
	cudaTextureObject_t voxel_data,	//< volume data
	float* ray_entry_d,				//< ray entry
	float* ray_exit_d,				//< ray exit
	float * noise_d,				//< noise for ray offsets 
	float samples_per_voxel,		//< samples per voxel
	cudaTextureObject_t tf			//< transfer function
	)
{
	dim3 block_size;
	block_size.x=16;
	block_size.y=16;
	dim3 grid_size;
	grid_size.x = iDivUp(n_u,block_size.x);
	grid_size.y = iDivUp(n_v,block_size.y);

	if (n_c==1)
		kernel_raycast<1><<<grid_size, block_size>>>(n_u, n_v, pixel_data_d, model_C_Pinv_d, voxel_data, ray_entry_d, ray_exit_d, noise_d, samples_per_voxel, tf);
	else if (n_c==3)
		kernel_raycast<3><<<grid_size, block_size>>>(n_u, n_v, pixel_data_d, model_C_Pinv_d, voxel_data, ray_entry_d, ray_exit_d, noise_d, samples_per_voxel, tf);
	else if (n_c==4)
		kernel_raycast<4><<<grid_size, block_size>>>(n_u, n_v, pixel_data_d, model_C_Pinv_d, voxel_data, ray_entry_d, ray_exit_d, noise_d, samples_per_voxel, tf);
	else std::cerr << __FILE__ << " : " << __LINE__ << " : invalid number of channels!" << std::endl;

	cudaDeviceSynchronize();
	cudaCheckState
}