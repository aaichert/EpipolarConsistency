// Created by A. Aichert on Tue Oct 4th 2016
#include <iostream>

#include <LibUtilsCuda/UtilsCuda.hxx>

#include <LibUtilsCuda/CudaMemory.h>
#include <LibUtilsCuda/CudaBindlessTexture.h>

#include <LibUtilsCuda/culaut/culaut.hxx>

template <int n_c>
__global__ void kernel_raycast_iso(
	unsigned short n_u, unsigned short n_v,
	float * pixel_data,				//< image data (RGBA)
	float * model_C_Pinv_h,			//< volume model transform (voxels to world)
	cudaTextureObject_t voxel_data,	//< volume data
	float * ray_entry,				//< ray entry
	float * ray_exit,				//< ray exit
	float * noise,					//< noise for ray offsets
	float   samples_per_voxel,		//< samples per voxel
	float   iso_val					//< iso value
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
	float ray_direction[3];
	culaut::xvcpy<float,float,3>(ray_direction,exit);
	culaut::xvsub<float,3>(ray_direction,entry);

	// Test if we have a valid ray and scale ray_direction to step size
	float ray_length=culaut::xvnorm2<float,3>(ray_direction);
	if (ray_length<1.0)
	{
		culaut::xvset<float,n_c>(pixel,0.0f);
		return;
	}

	// Volume is clipped
	{
		float first_sample=tex3D<float>(voxel_data,entry[0],entry[1],entry[2]);
		if (first_sample>iso_val)
		{
			culaut::xvset<float,n_c>(pixel,0.0f);
			pixel[0]=1.0f;
			if (n_c>=4) pixel[3]=1.0f;
			return;
		}
	}

	// Go to start position (with random offset) and sample with samples_per_voxel
	float voxel[3];
	float ray_direction_mm[3];
	culaut::xvscale<float,3>(ray_direction,1.0f/ray_length/samples_per_voxel);
	culaut::xvlincomb<float,float,3>(voxel,entry,1.0,ray_direction,noise[idx]);


	for (float a=0;a<ray_length;a+=1.0/samples_per_voxel)
	{
		culaut::xvadd<float,3>(voxel,ray_direction);
		float sample=tex3D<float>(voxel_data,voxel[0],voxel[1],voxel[2]);
		if (sample>iso_val)
		{
			// Hit refinement (go backwards in decreasing steps)
			culaut::xvscale<float,3>(ray_direction,0.5);
			culaut::xvsub<float,3>(voxel,ray_direction);
			for (int i=0;i<5;i++)
			{
				if (tex3D<float>(voxel_data,voxel[0],voxel[1],voxel[2])>iso_val)
				{
					culaut::xvscale<float,3>(ray_direction,0.5);
					culaut::xvsub<float,3>(voxel,ray_direction);
				}
				else
				{
					culaut::xvscale<float,3>(ray_direction,0.5);
					culaut::xvadd<float,3>(voxel,ray_direction);
				}
			}

			// visualize surface_normal
			float gradient[]={
						(tex3D<float>(voxel_data,voxel[0]+1.0f,voxel[1],voxel[2])-tex3D<float>(voxel_data,voxel[0]-1.0f,voxel[1],voxel[2])),
						(tex3D<float>(voxel_data,voxel[0],voxel[1]+1.0f,voxel[2])-tex3D<float>(voxel_data,voxel[0],voxel[1]-1.0f,voxel[2])),
						(tex3D<float>(voxel_data,voxel[0],voxel[1],voxel[2]+1.0f)-tex3D<float>(voxel_data,voxel[0],voxel[1],voxel[2]-1.0f))
					};
			culaut::xvnormalize2<float,3>(gradient);
			culaut::xvnormalize2<float,3>(ray_direction);
			float lambertian=culaut::xvdot<float,3>(gradient,ray_direction);
			if (lambertian<0.0f || (lambertian<0.0f&&lambertian>0.0f)) lambertian=0.0f;
			if (lambertian>1.0f) lambertian=1.0f;

			culaut::xvset<float,n_c>(pixel,lambertian);
			if (n_c>=3)
				culaut::xvcpy<float,float,3>(pixel,voxel);
			if (n_c>=6)
				culaut::xvcpy<float,float,3>(pixel+3,gradient);
			
			return;
		}
		
	}

	culaut::xvset<float,n_c>(pixel,0.0f);
}

// GPU ray casting. Iso-surface.
extern void raycast_iso(
	int n_u, int n_v, int n_c,		//< image size and number of channels
	float * pixel_data_d,			//< image data (RGBA)
	float * model_C_Pinv_d,			//< volume model transform (voxels to world)
	cudaTextureObject_t voxel_data,	//< volume data
	float * ray_entry_d,			//< ray entry
	float * ray_exit_d,				//< ray exit
	float * noise_d,				//< random ray offsets
	float   samples_per_voxel,		//< samples per voxel
	float   iso_val					//< iso value
	)
{
	dim3 block_size;
	block_size.x=16;
	block_size.y=16;
	dim3 grid_size;
	grid_size.x = iDivUp(n_u,block_size.x);
	grid_size.y = iDivUp(n_v,block_size.y);

	if (n_c==1) // shaded iso-surface
		kernel_raycast_iso<1><<<grid_size, block_size>>>(n_u, n_v, pixel_data_d, model_C_Pinv_d, voxel_data, ray_entry_d, ray_exit_d, noise_d, samples_per_voxel, iso_val);
	else if (n_c==3) // xyz surface point
		kernel_raycast_iso<3><<<grid_size, block_size>>>(n_u, n_v, pixel_data_d, model_C_Pinv_d, voxel_data, ray_entry_d, ray_exit_d, noise_d, samples_per_voxel, iso_val);
	else if (n_c==4) // xyz surface point and shading in alpha
		kernel_raycast_iso<4><<<grid_size, block_size>>>(n_u, n_v, pixel_data_d, model_C_Pinv_d, voxel_data, ray_entry_d, ray_exit_d, noise_d, samples_per_voxel, iso_val);
	else if (n_c==6) // xyz surface point and xyz normal
		kernel_raycast_iso<4><<<grid_size, block_size>>>(n_u, n_v, pixel_data_d, model_C_Pinv_d, voxel_data, ray_entry_d, ray_exit_d, noise_d, samples_per_voxel, iso_val);
	else if (n_c==7) //  xyz surface point and xyz normal and shading
		kernel_raycast_iso<4><<<grid_size, block_size>>>(n_u, n_v, pixel_data_d, model_C_Pinv_d, voxel_data, ray_entry_d, ray_exit_d, noise_d, samples_per_voxel, iso_val);
	else std::cerr << __FILE__ << " : " << __LINE__ << " : invalid number of channels!" << std::endl;


	cudaDeviceSynchronize();
	cudaCheckState
}
