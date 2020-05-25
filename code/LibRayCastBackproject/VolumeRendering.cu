// Created by A. Aichert on Tue Oct 4th 2016
#include <iostream>

#include <LibCudaUtils/CudaUtils.hxx>

#include <LibCudaUtils/CudaMemory.h>
#include <LibCudaUtils/CudaBindlessTexture.h>

#include <LibCudaUtils/culaut/culaut.hxx>
#include <LibCudaUtils/culaut/xprojectionmatrix.hxx>
#include <LibCudaUtils/culaut/xprojectivegeometry.hxx>

#include "ray_box_intersection.hxx"

// C must be dehomogenized if finite or normalized if infinite
__global__ void kernel_rayGeometryAABB(
	int n_u, int n_v,
	RayBoxIntersection::AxisAlignedBox<float> box,
	float * Pinv, float * C,
	float * ray_entry, float * ray_exit)
{
	// Find index of current thread
	int idx_x = blockIdx.x * blockDim.x + threadIdx.x;
	int idx_y = blockIdx.y * blockDim.y + threadIdx.y;
	if (idx_x>=n_u) return;
	if (idx_y>=n_v) return;
	int idx=idx_y*n_u+idx_x;

	// Backproject pixel
	float x[3]={(float)idx_x,(float)idx_y,1.0f};
	float X[4], Rx[6];
	culaut::xgemv<float,4,3>(Pinv,x,X);
	culaut::xvdehomogenize<float,4>(X);

	// Compute Plücker coordinates of ray
	culaut::join_pluecker(X,C,Rx);

	// Initialize entry and exit with zero
	float *entry=ray_entry+4*idx;
	float *exit =ray_exit+4*idx;

	for (int i=0;i<3;i++)
	{
		// Compute min-plane and test for intersections
		float E[4];
		E[0]=E[1]=E[2]=0;
		E[i]=1;
		E[3]=-box.aabb_min[i];
		// Intersect
		if (i==0) // first plane: initialize
		{
			culaut::meet_pluecker(Rx,E,X);
			culaut::xvcpy<float,float,4>(entry,X);
			culaut::xvcpy<float,float,4>(exit ,X);
		}
		else
		{
			// Test if we are inside correct half spaces of plane E
			bool entry_pass=culaut::xvdot<float,4>(entry,E)>=0;
			bool exit_pass =culaut::xvdot<float,4>(exit ,E)<=0;
			// If only exit or only entry fail the test, then update
			if (exit_pass && !entry_pass)
				culaut::meet_pluecker(Rx,E,entry);
			else if (entry_pass && !exit_pass)
				culaut::meet_pluecker(Rx,E,exit );
			else if (!entry_pass && !exit_pass)
			{
				// But if both fail, we have a miss
				culaut::xvset<float,4>(entry,0.0f);
				culaut::xvset<float,4>(exit ,0.0f);
				return;
			}
		}

		// Same for max-plane
		E[0]=E[1]=E[2]=0;
		E[i]=-1;
		E[3]=box.aabb_max[i];
		// Test if we are inside correct half spaces of plane E
		bool entry_pass=culaut::xvdot<float,4>(entry,E)>=0;
		bool exit_pass =culaut::xvdot<float,4>(exit ,E)<=0;
		// If only exit or only entry fail the test, then update
		if (exit_pass && !entry_pass)
			culaut::meet_pluecker(Rx,E,entry);
		else if (entry_pass && !exit_pass)
			culaut::meet_pluecker(Rx,E,exit );
		else if (!entry_pass && !exit_pass)
		{
			// But if both fail, we have a miss
			culaut::xvset<float,4>(entry,0.0f);
			culaut::xvset<float,4>(exit ,0.0f);
			return;
		}

	}
	culaut::xvdehomogenize<float,4>(entry);
	culaut::xvdehomogenize<float,4>(exit );

	//culaut::xvset4<float>(entry,(float)idx_x,0.0f,0.0f,1.f);
	//culaut::xvset4<float>(exit ,0.0f,(float)idx_y,0.0f,1.f);
}

// C must be dehomogenized if finite or normalized if infinite
void rayGeometryAABB(
	int n_u, int n_v,
	RayBoxIntersection::AxisAlignedBox<float> box,
	float * Pinv_d, float * C_d,
	float * out_ray_entry_d, float * out_ray_exit_d)
{
	dim3 block_size;
	block_size.x=16;
	block_size.y=16;
	dim3 grid_size;
	grid_size.x = iDivUp(n_u,block_size.x);
	grid_size.y = iDivUp(n_v,block_size.y);

	kernel_rayGeometryAABB<<<grid_size, block_size>>>(n_u, n_v, box, Pinv_d, C_d, out_ray_entry_d, out_ray_exit_d);
	cudaDeviceSynchronize();
	cudaCheckState
}




////////////
//
////////////

template <int raycast_mode>
__global__ void kernel_rayCast(
	int n_u, int n_v,				//< image size
	float * pixel_data,				//< image data (RGBA)
	float * model_C_Pinv_h,			//< volume model transform (voxels to world)
	cudaTextureObject_t voxel_data,	//< volume data
	float* ray_entry,				//< ray entry
	float* ray_exit,				//< ray exit
	const int    mode				//< Raycast mode (0: debug, 1: mip, 2: iso, 3: drr)
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
	float *pixel=pixel_data+4*idx;

	float ray_length=culaut::xvdistance2<float,3>(entry,exit);
	if (ray_length<=0)
	{
		culaut::xvset<float,4>(pixel,0);
		return;
	}

	float intensity=0;
	float step=1.0/ray_length;
	for (float a=0;a<1.0;a+=step)
	{
		float a_=1.0f-a;
		float p[3]={ a*entry[0]+a_*exit[0], a*entry[1]+a_*exit[1], a*entry[2]+a_*exit[2] };
		float sample=tex3D<float>(voxel_data,p[0],p[1],p[2]);
		if (raycast_mode==0)
		{
			if (intensity<sample) intensity=sample;
		}
		if (raycast_mode==1)
		{
			if (intensity<sample) intensity=sample;
		}
		if (raycast_mode==0)
		{
			if (intensity<sample) intensity=sample;
		}


	}

	// culaut::xvset<float,3>(pixel,ray_length);
	// pixel[4]=1;

	culaut::xvset4<float>(pixel,intensity,intensity,intensity,1.f);


//	culaut::xvset4<float>(pixel,(float)idx_x,(float)idx_y,0.0f,1.f);
}


void rayCast(
	int n_u, int n_v,				//< image size
	float * pixel_data_d,			//< image data (RGBA)
	float * model_C_Pinv_d,			//< volume model transform (voxels to world)
	cudaTextureObject_t voxel_data,	//< volume data
	float* ray_entry_d,				//< ray entry
	float* ray_exit_d,				//< ray exit
	int    mode						//< Raycast mode (0: debug, 1: mip, 2: iso, 3: drr)
	)
{
	dim3 block_size;
	block_size.x=16;
	block_size.y=16;
	dim3 grid_size;
	grid_size.x = iDivUp(n_u,block_size.x);
	grid_size.y = iDivUp(n_v,block_size.y);

	kernel_rayCast<<<grid_size, block_size>>>(n_u, n_v, pixel_data_d, model_C_Pinv_d, voxel_data, ray_entry_d, ray_exit_d, mode);
	cudaDeviceSynchronize();
	cudaCheckState
}