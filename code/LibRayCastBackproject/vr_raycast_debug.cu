// Created by A. Aichert on Tue Oct 4th 2016
#include <iostream>

#include <LibUtilsCuda/UtilsCuda.hxx>

#include <LibUtilsCuda/CudaMemory.h>
#include <LibUtilsCuda/CudaBindlessTexture.h>

#include <LibUtilsCuda/culaut/culaut.hxx>

template <int n_c>
__global__ void kernel_raycast_debug(
	unsigned short n_u, unsigned short n_v,
	float * pixel_data,				//< image data (RGBA)
	float * model_C_Pinv_h,			//< volume model transform (voxels to world)
	cudaTextureObject_t voxel_data,	//< volume data
	float * ray_entry,				//< ray entry
	float * ray_exit,				//< ray exit
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

	// if we have enough channels, we just wrte out the proxy geometry
	if (n_c>=6)
	{
		culaut::xvcpy<float,float,3>(pixel  ,entry);
		culaut::xvcpy<float,float,3>(pixel+3,exit );
		return;
	}

	// Otherwise we show a little bit of everything for debugging purposes.
	float ray_length=culaut::xvdistance2<float,3>(entry,exit);
	if (ray_length<=0)
	{
		culaut::xvset<float,n_c>(pixel,0.0f);
		return;
	}
	if (idx_x<n_u*0.5)
	{
		if (idx_y<n_v*0.5)
		{
			if (n_c>=3)
				culaut::xvcpy<float,float,3  >(pixel,entry);
			else
				culaut::xvcpy<float,float,n_c>(pixel,entry);
		}
		else
		{
			if (n_c>=3)
				culaut::xvcpy<float,float,3  >(pixel,exit );
			else
				culaut::xvcpy<float,float,n_c>(pixel,exit );
		}
		if (n_c>3) pixel[3]=samples_per_voxel;
	}
	else
	{
		if (idx_y>n_v*0.5)
		{
			float intensity=0;
			float step=samples_per_voxel/ray_length;
			float voxel[3];
			for (float a=0;a<1.0;a+=step)
			{
				culaut::xvlincomb<float,float,3>(voxel,entry,1.0-a,exit,a);
				float sample=tex3D<float>(voxel_data,voxel[0],voxel[1],voxel[2]);
				if (intensity<sample) intensity=sample;
			}
			culaut::xvset<float,n_c>(pixel,intensity);
		}
		else
		{
			culaut::xvset<float,n_c>(pixel,0.0f);
			pixel[0]=(float)idx_x;
			if (n_c>=2) pixel[1]=(float)idx_y;
		}
	}
}


void raycast_debug(
	int n_u, int n_v, int n_c,		//< image size and number of channels
	float * pixel_data_d,			//< image data (RGBA)
	float * model_C_Pinv_d,			//< volume model transform (voxels to world)
	cudaTextureObject_t voxel_data,	//< volume data
	float * ray_entry_d,			//< ray entry
	float * ray_exit_d,				//< ray exit
	float   samples_per_voxel		//< samples per voxel
	)
{
	dim3 block_size;
	block_size.x=16;
	block_size.y=16;
	dim3 grid_size;
	grid_size.x = iDivUp(n_u,block_size.x);
	grid_size.y = iDivUp(n_v,block_size.y);

	if (n_c==1)
		kernel_raycast_debug<1><<<grid_size, block_size>>>(n_u, n_v, pixel_data_d, model_C_Pinv_d, voxel_data, ray_entry_d, ray_exit_d, samples_per_voxel);
	else if (n_c==3)
		kernel_raycast_debug<3><<<grid_size, block_size>>>(n_u, n_v, pixel_data_d, model_C_Pinv_d, voxel_data, ray_entry_d, ray_exit_d, samples_per_voxel);
	else if (n_c==4)
		kernel_raycast_debug<4><<<grid_size, block_size>>>(n_u, n_v, pixel_data_d, model_C_Pinv_d, voxel_data, ray_entry_d, ray_exit_d, samples_per_voxel);
	else if (n_c==6)
		kernel_raycast_debug<4><<<grid_size, block_size>>>(n_u, n_v, pixel_data_d, model_C_Pinv_d, voxel_data, ray_entry_d, ray_exit_d, samples_per_voxel);
	else std::cerr << __FILE__ << " : " << __LINE__ << " : invalid number of channels!" << std::endl;

	cudaDeviceSynchronize();
	cudaCheckState
}