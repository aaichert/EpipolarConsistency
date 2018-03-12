
// Created by A. Aichert on Fr Dec 15th 2017
#include <iostream>

#include <LibUtilsCuda/cutil_math.h>
#include <LibUtilsCuda/CudaMemory.h>
#include <LibUtilsCuda/UtilsCuda.hxx>

#include <LibUtilsCuda/culaut/culaut.hxx>

__constant__ float eps=1E-6;

// make sure a point is within rectangle [0,0]-[n_x n_y]
__device__ inline bool inBounds(float x, float y, float n_x, float n_y)
{
	if (x<=n_x&&y<=n_y&&x>=0&&y>=0) return true;
	return false;
}

// A basic sorting algorithm.
__device__ inline void sort4(float *v)
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

// Transform image into radon space and coppute angular derivative
__global__ void kernel_computeLineIntegrals(float * lines, float *abcd, int n_lines, cudaTextureObject_t tex, float n_x, float n_y, float *out, float baseline_distance, bool derivative)
{
	// Find index of current thread
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx>=n_lines) return;

	// The line
	float l[]={lines[idx*3+0], lines[idx*3+1], lines[idx*3+2]};

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
			(n_x-1-o[0])/d[0],
			(    1-o[1])/d[1],
			(n_y-1-o[1])/d[1]
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
	if (!inBounds(o[0]+t_min*d[0],o[1]+t_min*d[1],n_x,n_y))
	{
		out[idx]=0;
		return;
	}

	// Shift line origin since pixel[0][0] is at tex2D(.5,.5)
	o[0]+=.5f;
	o[1]+=.5f;

	// Jacobian of projective transformation (optional)
	struct LinePerspectivity {
		float a;
		float b;
		float c;
		float d;

		/// From raw memory
		__device__ LinePerspectivity(float *v=0x0) : a(v?v[0]:1.f) , b(v?v[1]:.0f) , c(v?v[2]:.0f) , d(v?v[3]:1.f) {}	

		/// Perspective transformation of line coordinate
		__device__ float transform (float t)       const {
			return (a*t+b)/(c*t+d);
		}

		/// Inverse perspective transformation of line coordinate
		__device__ float inverse   (float t_prime) const {
			return (d*t_prime-b)/(-c*t_prime+a);
		}

		/// Derivative of transform(...)
		__device__ float derivative(float t)       const {
			return (a*d-b*c) / (c*c*t*t +2*c*d*t +d*d);
		}

	};

	// The perspectivity on the line.
	float *abcd_idx  =abcd?abcd+7*idx:0x0;
	LinePerspectivity phi(abcd_idx);
	float dC_Ak_px   =abcd_idx?abcd_idx[4]:0.f;
	float t_prime_ak =abcd_idx?abcd_idx[5]:0.f;
	float v          =abcd_idx?abcd_idx[6]:0.f;

	// Pixel step in image for raycasting.
	const float step=0.4;

	// ECC 2do implement with warping for direct comparison
	if (derivative)
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
	else 
	{
		if (abcd)
		{
			// Rectified by Weighting
			float sum=0;
			//float blubb=sqrtf(v*v+baseline_distance*baseline_distance);
			for (float t=t_min; t<=t_max; t+=step)
			{
				// Compute Derivative of perspectivity phi
				float jphi=phi.derivative(t);
				// Compute cosine of angle between ray and line on virtual detector
				float t_prime_ak_diff=phi.transform(t)-t_prime_ak;
				float fan_beam_cc_weight=1.0/sqrtf(t_prime_ak_diff*t_prime_ak_diff+dC_Ak_px*dC_Ak_px+v*v+baseline_distance*baseline_distance);
				// Weighted Integral
				sum+=jphi*tex2D<float>(tex,o[0]+t*d[0],o[1]+t*d[1])*step*fan_beam_cc_weight;
			}
			out[idx]=sum;
		}
		else
		{
			// Plain Integral
			float sum=0;
			for (float t=t_min; t<=t_max; t+=step) {
				sum+=tex2D<float>(tex,o[0]+t*d[0],o[1]+t*d[1])*step;
			}
			out[idx]=sum;
		}
	}


}

//derivative
//plain integral
//rectified
//warping for validation can be used by setting rectified and derivative (which does not other wise make sense.)

void cuda_computeLineIntegrals(float* lines_d, float *abcd_d, int n_lines, cudaTextureObject_t I, int n_x, int n_y, float *integrals_d, float baseline_distance, bool derivative)
{
	cudaCheckState
	// Threads per block and problem size
	dim3 block_size;
	block_size.x=32;
	dim3 grid_size;
	grid_size.x = iDivUp(n_lines,block_size.x);

	// Start kernel (radon transform)
	kernel_computeLineIntegrals<<<grid_size, block_size>>>(lines_d, abcd_d, n_lines, I, (float)n_x, (float)n_y, integrals_d, baseline_distance, derivative);
	cudaDeviceSynchronize();
}


#include <LibUtilsCuda/culaut/culaut.hxx>

__global__ void kernel_warp_1c(culaut::Array<float,9> H_inv, int n_x_in, int n_y_in, cudaTextureObject_t input_image, int n_x_out, int n_y_out, float* output_d)
{
	// Find index of current thread
	int idx_x = blockIdx.x * blockDim.x + threadIdx.x;
	int idx_y = blockIdx.y * blockDim.y + threadIdx.y;
	if (idx_x>=n_x_out) return;
	if (idx_y>=n_y_out) return;
	int idx=idx_y*n_x_out+idx_x;

	float x_out[]={(float)idx_x, (float)idx_y, 1.0f};
	float x_in[3];
	culaut::xgemm<float,3,3,3>(H_inv.v,x_out,x_in);
	culaut::xvdehomogenize<float,3>(x_in);
	output_d[idx]=tex2D<float>(input_image,x_in[0]+.5f,x_in[1]+.5f);
}

void cuda_warp_image_2d1c(culaut::Array<float,9> H_inv, int n_x_in, int n_y_in, cudaTextureObject_t input_image, int n_x_out, int n_y_out, float* output_d)
{
	dim3 block_size;
	block_size.x=16;
	block_size.y=16;
	dim3 grid_size;
	grid_size.x = iDivUp(n_x_out,block_size.x);
	grid_size.y = iDivUp(n_y_out,block_size.y);

	kernel_warp_1c<<<grid_size, block_size>>>(H_inv, n_x_in, n_y_in, input_image,  n_x_out, n_y_out, output_d);
	cudaDeviceSynchronize();
	cudaCheckState
}

__global__ void kernel_horiz_integral(culaut::Array<float,9> H_inv, int n_x_in, int n_y_in, cudaTextureObject_t input_image, float n_x_out, float n_y_out, float* output_d)
{
	// Find index of current thread
	int idx_y = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx_y>=n_y_out) return;


	float sum=0;
	for (float idx_x=0;idx_x<n_x_out;idx_x++)
	{
		float x_out[]={(float)idx_x, (float)idx_y, 1.0f};
		float x_in[3];
		culaut::xgemm<float,3,3,3>(H_inv.v,x_out,x_in);
		culaut::xvdehomogenize<float,3>(x_in);
		sum+=tex2D<float>(input_image,x_in[0]+.5f,x_in[1]+.5f);;
	}

	output_d[idx_y]=sum;
}

void cuda_horiz_integral(culaut::Array<float,9> H_inv, int n_x_in, int n_y_in, cudaTextureObject_t input_image, int n_x_out, int n_y_out, float* output_d)
{
	dim3 block_size;
	block_size.x=64;
	dim3 grid_size;
	grid_size.x = iDivUp(n_y_out,block_size.x);

	kernel_horiz_integral<<<grid_size, block_size>>>(H_inv, n_x_in, n_y_in, input_image,  (float)n_x_out, (float)n_y_out, output_d);
	cudaDeviceSynchronize();
	cudaCheckState
}
