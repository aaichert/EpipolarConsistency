// Created by A. Aichert on Tue Oct 4th 2016
#include <iostream>

#include <LibUtilsCuda/UtilsCuda.hxx>

#include <LibUtilsCuda/CudaMemory.h>
#include <LibUtilsCuda/CudaBindlessTexture.h>

#include <LibUtilsCuda/culaut/culaut.hxx>
#include <LibUtilsCuda/culaut/xprojectionmatrix.hxx>
#include <LibUtilsCuda/culaut/xprojectivegeometry.hxx>

#include <vector>

#include "vr_cuda.hxx"


////////////
// Axis aligned bounding box geometry
////////////

// C must be dehomogenized if finite or normalized if infinite
__global__ void kernel_rayGeometryAABB(
	int n_u, int n_v,
	AxisAlignedBox<float> box,
	float * Pinv, float * C,
	float * ray_entry, float * ray_exit,
	int n_clip_planes, float * clip_planes)
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
	float entry[]={0.0f, 0.0f, 0.0f, 0.0f};
	float exit []={0.0f, 0.0f, 0.0f, 0.0f};

	//culaut::xvcpy<float,float,3>(entry,C);
	//culaut::xvcpy<float,float,3>(exit ,C);

	for (int i=0;i<3;i++)
	{
		// Compute min-plane and test for intersections
		float E[4];
		E[0]=E[1]=E[2]=0;
		E[i]=1;
		E[3]=-box.aabb_min[i];
		{
			// Test if we are inside correct half spaces of plane E
			bool entry_pass=culaut::xvdot<float,4>(entry,E)>=0;
			bool exit_pass =culaut::xvdot<float,4>(exit ,E)<=0;
			// If only exit or only entry fail the test, then update
			if (exit_pass && !entry_pass)
				culaut::meet_pluecker(Rx,E,entry);
			else if (entry_pass && !exit_pass)
				culaut::meet_pluecker(Rx,E,exit );
			else if (i==0)
			{
				culaut::meet_pluecker(Rx,E,entry);
				culaut::meet_pluecker(Rx,E,exit );			
			}
			else if (!entry_pass && !exit_pass)
			{
				// But if both fail, we have a miss
				culaut::xvset<float,3>(ray_entry+4*idx,0.0f);
				culaut::xvset<float,3>(ray_exit +4*idx,0.0f);
				return;
			}
		}

		// Same for max-plane
		E[0]=E[1]=E[2]=0;
		E[i]=-1;
		E[3]=box.aabb_max[i];
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
				culaut::xvset<float,3>(ray_entry+4*idx,0.0f);
				culaut::xvset<float,3>(ray_exit +4*idx,0.0f);
				return;
			}
		}

	}

	if (clip_planes)
	{
		for (int i=0;i<n_clip_planes;i++)
		{
			float *E=clip_planes+i*4;
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
				culaut::xvset<float,3>(ray_entry+4*idx,0.0f);
				culaut::xvset<float,3>(ray_exit +4*idx,0.0f);
				return;
			}
		}
	}

	culaut::xvdehomogenize<float,4>(entry);
	culaut::xvdehomogenize<float,4>(exit );

	bool C_in_box=
		C[0]>=box.aabb_min[0] && C[0]<=box.aabb_max[0] &&
		C[1]>=box.aabb_min[1] && C[1]<=box.aabb_max[1] &&
		C[2]>=box.aabb_min[2] && C[2]<=box.aabb_max[2];
	// alternative: test if exit is C!

	if (C_in_box)
	{
		// Ray goes from C to exit (because plane normals are reversed w.r.t. to inside)
		culaut::xvcpy<float,float,3>(ray_entry+4*idx, C);
		culaut::xvcpy<float,float,3>(ray_exit+4*idx , entry);
	}
	else
	{
		culaut::xvcpy<float,float,3>(ray_entry+4*idx, entry);
		culaut::xvcpy<float,float,3>(ray_exit+4*idx , exit);
	}

	//culaut::xvset4<float>(ray_entry+4*idx,(float)idx_x,0.0f,0.0f,1.f);
	//culaut::xvset4<float>(ray_exit +4*idx ,0.0f,(float)idx_y,0.0f,1.f);
}

// C must be dehomogenized if finite or normalized if infinite
void rayGeometryAABB(
	int n_u, int n_v,
	AxisAlignedBox<float> box,
	float * Pinv_d, float * C_d,
	float * out_ray_entry_d, float * out_ray_exit_d,
	int n_clip_planes=0, float * clip_planes_d=0x0)
{
	dim3 block_size;
	block_size.x=16;
	block_size.y=16;
	dim3 grid_size;
	grid_size.x = iDivUp(n_u,block_size.x);
	grid_size.y = iDivUp(n_v,block_size.y);

	kernel_rayGeometryAABB<<<grid_size, block_size>>>(n_u, n_v, box, Pinv_d, C_d, out_ray_entry_d, out_ray_exit_d, n_clip_planes, clip_planes_d);
	cudaDeviceSynchronize();
	cudaCheckState
}


////////////
// Update using min/max binning ESS
////////////

__device__ 
inline void ess(float *ray_entry, float *ray_exit, float bin_size, cudaTextureObject_t binned_minmax, float value_min, float value_max)
{
	// Get original values andscale to binned volume voxels
	float bin_entry[3];
	float bin_exit [3];
	culaut::xvcpy<float,float,3>(bin_entry,ray_entry);
	culaut::xvcpy<float,float,3>(bin_exit ,ray_exit);
	culaut::xvscale<float,3>(bin_entry,1.0f/bin_size);
	culaut::xvscale<float,3>(bin_exit ,1.0f/bin_size);

	// ray_direction=exit-entry
	float ray_direction[3];
	culaut::xvsub<float,float,3>(ray_direction,bin_exit,bin_entry);

	// Test if we have a valid ray and scale ray_direction to step size
	float ray_length=culaut::xvnorm2<float,3>(ray_direction);
	if (ray_length<1e-8)
	{
		culaut::xvset<float,3>(ray_entry,0.0f);
		culaut::xvset<float,3>(ray_exit,0.0f);
		return;
	}
	
	// Find voxel step with half binsize as sampling distance
	const float voxels_per_sample=0.5f;
	culaut::xvscale<float,3>(ray_direction,voxels_per_sample/ray_length);

	// Search for ray entry in binned volume
	float voxel[3];
	culaut::xvcpy<float,float,3>(voxel,bin_entry);
	bool entry_found=false;
	for (float a=0;a<ray_length;a+=voxels_per_sample)
	{
		float2 minmax=tex3D<float2>(binned_minmax,voxel[0],voxel[1],voxel[2]);
		culaut::xvadd<float,3>(voxel,ray_direction);
		if (minmax.y>value_min && minmax.x<value_max)
		{
			// Found ray entry!
			culaut::xvsub<float,3>(voxel,ray_direction);
			culaut::xvscale<float,3>(voxel,bin_size);
			culaut::xvcpy<float,float,3>(ray_entry,voxel);
			entry_found=true;
			break;
		}
	}

	// Invalidate pixels where the value range was not hit at all.
	if (!entry_found)
	{
		culaut::xvset<float,3>(ray_entry,0.0f);
		culaut::xvset<float,3>(ray_exit ,0.0f);
		return;
	}

	// Backward search for exit
	culaut::xvcpy<float,float,3>(voxel,bin_exit);
	for (float a=ray_length;a>=ray_length;a-=voxels_per_sample)
	{
		float2 minmax=tex3D<float2>(binned_minmax,voxel[0],voxel[1],voxel[2]);
		if (minmax.y>value_min && minmax.x<value_max)
		{
			// Found ray exit!
			culaut::xvadd<float,3>(voxel,ray_direction);
			culaut::xvscale<float,3>(voxel,bin_size);
			culaut::xvcpy<float,float,3>(ray_exit,voxel);
		}
		culaut::xvsub<float,3>(voxel,ray_direction);
	}

}

__global__ void kernel_rayGeometryESS(
	int n_u, int n_v,
	float * ray_entry, float * ray_exit,
	float bin_size, cudaTextureObject_t binned_minmax,
	float value_min, float value_max
	)
{
	// Find index of current thread
	int idx_x = blockIdx.x * blockDim.x + threadIdx.x;
	int idx_y = blockIdx.y * blockDim.y + threadIdx.y;
	if (idx_x>=n_u) return;
	if (idx_y>=n_v) return;
	int idx=idx_y*n_u+idx_x;

	ess(ray_entry+4*idx,ray_exit +4*idx, bin_size, binned_minmax, value_min, value_max);

}

// GPU ray geometry update (optimized boxes)
void rayGeometryESS(
	int n_u, int n_v,
	float * out_ray_entry_d, float * out_ray_exit_d,
	int bin_size, cudaTextureObject_t binned_minmax,
	float value_min, float value_max
	)
{
	dim3 block_size;
	block_size.x=16;
	block_size.y=16;
	dim3 grid_size;
	grid_size.x = iDivUp(n_u,block_size.x);
	grid_size.y = iDivUp(n_v,block_size.y);

	kernel_rayGeometryESS<<<grid_size, block_size>>>(n_u, n_v, out_ray_entry_d, out_ray_exit_d, (float)bin_size, binned_minmax, value_min, value_max);
	cudaDeviceSynchronize();
	cudaCheckState
}
