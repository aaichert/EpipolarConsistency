// Created by A. Aichert on Fr Nov 29th 2013. Rewrite Tue Dec 19th 2017.
#include <iostream>

#include <LibUtilsCuda/CudaMemory.h>

#include <cufft.h>

__constant__ float Pi=3.14159265359;

// Make sure a point is within rectangle [0,0]-[n_u n_v]
__device__ inline bool inBounds(float u, float v, float n_u, float n_v)
{
	if (u<=n_u&&v<=n_v&&u>=0&&v>=0) return true;
	return false;
}

// A basic sorting algorithm of four values.
__device__ __host__ inline void sort4(float *v)
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

// Transform image into Radon space and coppute angular derivative
template <bool derivative>
__global__ void radonDerivative(cudaTextureObject_t tex, float n_u, float n_v, float *out, int n_alpha, int n_t, short post_process)
{
	// Compute bin index in image of Radon transform and corresponding line
	int   idx;
	float o[2];
	float d[2];
	{
		float l[3];
		int ix = blockIdx.x * blockDim.x + threadIdx.x;
		int iy = blockIdx.y * blockDim.y + threadIdx.y;
		if (ix>=n_alpha) return;
		if (iy>=n_t) return;
		idx=iy*n_alpha+ix;
		// Find index and relative location in range [-0.5, 0.5]^2
		float x_rel=(ix/(float)n_alpha-0.5f);
		float y_rel=(iy/(float)n_t    -0.5f);
		// Compute image diagonal d
		float diag=sqrtf(n_u*n_u+n_v*n_v);
		// Find line parameters
		float alpha=x_rel*Pi;    // angle alpha between -Pi/2 and Pi/2
		float tau  =y_rel*diag;  // distance t  between  -d/2 and  d/2
		// The line in Hessian normal form
		l[0]=-sinf(alpha);
		l[1]= cosf(alpha);
		l[2]=-tau;
		// Move coordinate origin to image center
		l[2]+=-0.5f*n_u*l[0]-0.5f*n_v*l[1];
		// Establish 1D line coordinate system (line coordinate t)
		// Line Origin : Closest point to the origin (t=0)
		o[0]=-l[2]*l[0];
		o[1]=-l[2]*l[1];
		// Line direction
		d[0]=l[1];
		d[1]=-l[0];
	}

	// Compute range over which the line intersects the image.
	float t,t_max;
	{
		float ts[]={
			(    1.f-o[0])/d[0],
			(n_u-1.f-o[0])/d[0],
			(    1.f-o[1])/d[1],
			(n_v-1.f-o[1])/d[1]
		};
		// Avoid Inf/NaN in case of vertical and horizontal lines.
		if (d[0]*d[0]<1e-12f) ts[0]=-(ts[1]=1e10f);
		if (d[1]*d[1]<1e-12f) ts[2]=-(ts[3]=1e10f);
		// Sorting - the middle two correspond to image edges.
		sort4(ts);
		t=ts[1];
		t_max=ts[2];		
	}

	__syncthreads();

	// Early exit if point for t_min (and hence no other) is within bounds
	if (!inBounds(o[0]+t*d[0],o[1]+t*d[1],n_u,n_v) || t_max <= t) {
		out[idx]=0;
		return;
	}

	//out[idx]=(t_max-t);
	//return;

	// Shift line origin since pixel[0][0] is at tex2D(.5,.5)
	o[0]+=.5f;
	o[1]+=.5f;

	// Compute line integral (or derivative thereof)
	const float step=.66;
	float sum=0;
	if (!derivative)
	{
		// Start summation over a line
		for (; t<=t_max; t+=step)
			sum +=tex2D<float>(tex,o[0]+t*d[0]     ,o[1]+t*d[1]     );
		out[idx]=sum*step;
	}
	else
	{
		// Move by half pixel in normal direction
		o[0]-=.5f*d[1];
		o[1]+=.5f*d[0];
		// Start summation over two parallel lines
		float sumo=0;
		for (; t<=t_max; t+=step)
		{
			sum +=tex2D<float>(tex,o[0]+t*d[0]     ,o[1]+t*d[1]     );
			// Offset by one pixel in negative normal direction
			sumo+=tex2D<float>(tex,o[0]+t*d[0]+d[1],o[1]+t*d[1]-d[0]);
		}
		// and return difference (approximation to derivative)
		if (post_process==1) {
			float result=(sum-sumo)*step;
			if (result<0)
				out[idx]=-sqrt(-result);
			else
				out[idx]=sqrt(result);
		}
		else if (post_process==2) {
			float result=(sum-sumo)*step;
			if (result<0)
				out[idx]=-log(-result+1);
			else
				out[idx]=log(result+1);
		}
		else
			out[idx]=(sum-sumo)*step;
	}

}

// Predcl of ramp filter for Smith DCC
void apply1DRampFilter(float* radon_transform_d, int n_alpha, int n_t);

/// Computed Radon intermediate function and returns n_t*n_alpha array in out_d (device array). Applied filter (Derivative=0, Ramp=1, None=2)
void computeDerivLineIntegrals(cudaTextureObject_t in, int n_u, int n_v, int n_alpha, int n_t, int filter, int post_process, float *out_d)
{
	cudaCheckState
	// Threads per block and problem size
	dim3 block_size;
	block_size.x=32;
	block_size.y=4;
	dim3 grid_size;
	grid_size.x = iDivUp(n_alpha,block_size.x);
	grid_size.y = iDivUp(n_t,block_size.y);
	// Start kernel (radon transform)
	if (filter==0)
		radonDerivative<true><<<grid_size, block_size>>>(in, (float)n_u, (float)n_v, out_d, n_alpha, n_t, (short)post_process);
	else
		radonDerivative<false><<<grid_size, block_size>>>(in, (float)n_u, (float)n_v, out_d, n_alpha, n_t, (short)post_process);
	cudaDeviceSynchronize();
	cudaCheckState

	if (filter==1)
		apply1DRampFilter(out_d, n_alpha, n_t);

}

// Multiply ramp and scale
__global__ void ramp_filter1D(cufftComplex *ftrt, int n_alpha, int n_theta, float scale)
{
	int ix = blockIdx.x * blockDim.x + threadIdx.x;
	int iy = blockIdx.y * blockDim.y + threadIdx.y;
	if (ix>=n_alpha) return;
	if (iy>=n_theta) return;
	int idx=iy*n_alpha+ix;

	ftrt[idx].x*=iy*scale;
	ftrt[idx].y*=iy*scale;
}

///
void apply1DRampFilter(float* radon_transform_d, int n_alpha, int n_t)
{
	cudaCheckState

    // Device side output data allocation
    cufftComplex *fourier_transforms_of_radon_transform_columns_d;
	int n_theta=n_t/2 + 1;
	cudaMalloc((void**)&fourier_transforms_of_radon_transform_columns_d, n_theta*n_alpha*sizeof(cufftComplex));
	cudaCheckState

	// n_alpha batched 1D FFTs over n_t elemenets in the SLOW image direction (hence a stride of n_alpha);
	cufftHandle plan;
	cufftPlanMany(&plan,
		1, &n_t,              // request 1D FFTs of size n_t
		&n_t, n_alpha, 1,     // stride and distance between batches (input) ; in this case interleaved storage, so stride is larger than batch distance.
		&n_theta, n_alpha, 1, // stride and distance between batches (output); in this case interleaved storage, so stride is larger than batch distance.
		CUFFT_R2C,            // we convert real to complex
		n_alpha);             // number of FFTs
	cudaCheckState
    cufftExecR2C(plan, radon_transform_d, fourier_transforms_of_radon_transform_columns_d);
	cudaDeviceSynchronize();
    cufftDestroy(plan);
	cudaCheckState

	// Threads per block and problem size
	dim3 block_size;
	block_size.x=8;
	block_size.y=32;
	dim3 grid_size;
	grid_size.x = iDivUp(n_alpha,block_size.x);
	grid_size.y = iDivUp(n_theta,block_size.y);

	// Kernel for multiplication with ramp and appropriate scaling.
	ramp_filter1D<<<grid_size, block_size>>>(fourier_transforms_of_radon_transform_columns_d, n_alpha, n_theta, -0.5f/(n_t*n_theta) );
	cudaCheckState

	// Execute inverse FFTs
	cufftPlanMany(&plan,
		1, &n_t,              // request 1D FFTs of size n_t
		&n_theta, n_alpha, 1, // stride and distance between batches (input) ; in this case interleaved storage, so stride is larger than batch distance.
		&n_t, n_alpha, 1,     // stride and distance between batches (output); in this case interleaved storage, so stride is larger than batch distance.
		CUFFT_C2R,            // we convert real to complex
		n_alpha);             // number of FFTs
	cudaCheckState

	// Clean up
	cufftExecC2R(plan, fourier_transforms_of_radon_transform_columns_d, radon_transform_d);
	cudaDeviceSynchronize();
    cufftDestroy(plan);
    cudaFree(fourier_transforms_of_radon_transform_columns_d);
	cudaCheckState
}

