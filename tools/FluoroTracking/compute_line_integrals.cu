
// Created by A. Aichert on Wed Jan 10th 2018
#include <iostream>

#include <LibUtilsCuda/CudaMemory.h>
#include <LibUtilsCuda/UtilsCuda.hxx>

#include <LibEpipolarConsistency/RectifiedFBCC.h>


// Make sure a point is within rectangle [0,0]-[n_u n_v]
__device__ inline bool inBounds(float u, float v, float n_u, float n_v)
{
	if (u<=n_u&&v<=n_v&&u>=0&&v>=0) return true;
	return false;
}

// A basic sorting algorithm of four values.
__device__ __host__ void sort4(float *v)
{
	for (int j=0;j<3;j++)
		for (int i=0;i<3;i++)
			if (v[i]>v[i+1])
			{
				float tmp=v[i];
				v[i]=v[i+1];
				v[i+1]=tmp;
			}
}

// Compute ECC or FBCC for certain epipolar lines on the image.
__global__ void kernel_computeLineIntegrals(
		float* lines, short line_stride, short n_lines,
		float *fbcc_d, short fbcc_stride,
		cudaTextureObject_t tex, short n_u, short n_v,
		float *out)
{
	// Find index of current thread
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx>=n_lines) return;

	// The line
	float l[]={lines[idx*line_stride+0], lines[idx*line_stride+1], lines[idx*line_stride+2]};

	// Establish 1D line coordinate system (line coordinate t)
	// Line Origin : Closest point to the origin (t=0)
	float o[]={-l[2]*l[0],-l[2]*l[1]};

	// Line direction
	float d[]={l[1],-l[0]};

	// Compute range at which the line intersects the image
	float t_min,t_max;
	{
		float ts[]={
			(    1-o[0])/d[0],
			(n_u-1-o[0])/d[0],
			(    1-o[1])/d[1],
			(n_v-1-o[1])/d[1]
		};
		// Avoid Inf/NaN
		if (d[0]*d[0]<1e-12) ts[0]=-(ts[1]=1e10f);
		if (d[1]*d[1]<1e-12) ts[2]=-(ts[3]=1e10f);
		// Sort and the middle two are image edges
		sort4(ts);
		t_min=ts[1];
		t_max=ts[2];		
	}

	// Early exit if point for t_min (and hence no other) is within bounds or line was invalid
	if (!inBounds(o[0]+t_min*d[0],o[1]+t_min*d[1],n_u,n_v))
	{
		out[idx]=0;
		return;
	}

	// Pixel step in image for raycasting.
	const float step=0.4;

	// Account for half-pixel offset when sampling textures
	o[0]+=.5f;
	o[1]+=.5f;

	if (fbcc_d)
	{
		FBCC_weighting_info fbcc=*((FBCC_weighting_info*)(fbcc_d+fbcc_stride*idx));
		// Rectified by Weighting
		float sum=0;
		for (float t=t_min; t<=t_max; t+=step)
		{
			// Compute virtual detector's u-coordinate relative to closest point on line to source
			float u_prime=fbcc.phi.transform(t)-fbcc.t_prime_ak;
			float fbcc_weight=1.0/sqrtf(u_prime*u_prime+fbcc.d_l_kappa_C_sq);
			float Jphi=fbcc.phi.derivative(t);
			// Weighted integral
			sum+=step*tex2D<float>(tex,o[0]+t*d[0],o[1]+t*d[1])*Jphi*fbcc_weight;
		}
		out[idx]=sum;
	}
	else
	{
		// Usual ECC. Numeric differenciation in direction l[0],l[1]
		l[0]*=0.5f;
		l[1]*=0.5f;
		// Start summation over two parallel lines
		float sump=0, summ=0;
		for (float t=t_min; t<=t_max; t+=step)
		{
			sump+=tex2D<float>(tex,o[0]+t*d[0]+l[0],o[1]+t*d[1]+l[1])*step;
			summ+=tex2D<float>(tex,o[0]+t*d[0]-l[0],o[1]+t*d[1]-l[1])*step;
		}
		// and return difference (approximation to derivative)
		out[idx]=(sump-summ);
	}
}

// Compute ECC or FBCC for certain epipolar lines on the image.
void cuda_computeLineIntegrals(
	float* lines_d, short line_stride, short n_lines,  // Lines in Hessian normal form, number of float values to next line and number of lines
	float *fbcc_d, short fbcc_stride,                  // Optional: FBCC_weighting_info
	cudaTextureObject_t I, short n_u, short n_v,       // The image and its size
	float *integrals_out_d)                            // Output memory: the integrals (size is n_lines)
{
	// Threads per block and problem size
	dim3 block_size;
	block_size.x=32;
	dim3 grid_size;
	grid_size.x = iDivUp(n_lines,block_size.x);
	// Launch kernel (radon transform)
	kernel_computeLineIntegrals<<<grid_size, block_size>>>(
			lines_d, line_stride, n_lines,
			fbcc_d, fbcc_stride,
			I, n_u, n_u,
			integrals_out_d
		);
	cudaDeviceSynchronize();
	cudaCheckState
}
