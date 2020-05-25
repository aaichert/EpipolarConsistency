// Created by A. Aichert on Fr Nov 29th 2013
#include <iostream>

#include <LibUtilsCuda/UtilsCuda.hxx>

#include <LibUtilsCuda/CudaBindlessTexture.h>
#include <LibUtilsCuda/CudaMemory.h>

#include <NRRD/nrrd_image_view.hxx>

#include "EpipolarConsistencyCommon.hxx"

__global__ void kernelEpipolarConsistencyComputeK01(
		float	n_x2,				//< Half width of images in pixels
		float	n_y2,				//< Half height of images in pixels
		int		num_dtrs,			//< Number of projection images (=# dtrs)
		int		num_Ps,				//< Number of projection matrices
		float*	Cs,					//< Source positions 4 * num_Ps, null spaces of projection matrices
		float*	PinvTs,				//< Projection matrix inverse transpose, column major 3x4 * num_Ps
		int		num_pairs,			//< Explicit assigment of projection matrices and dtrs. Can be zero.
		int*	indices,			//< num_indices*4 (P0,P1,dtr0,dtr1). Can be null. If not provided, all n*(n-1)*2 pairs are computed.
		float*	K01s,				//< out: pre-computed mapping from 3D plane angles to 2D lines
		float	step_alpha,			//< Angular spacing of radon transform
		float	step_t,				//< Line distance spacing of radon transform
		float	object_radius_mm,	//< Approximate size of object in mm (determines angular range)
		float	num_samples,		//< Number of samples to be drawn
		float	dkappa=0			//< Optional: fixed epipolar plane angle
		)
{
	// Find index of current thread
	int idx_in = blockIdx.x * blockDim.x + threadIdx.x;
	int idx_P0, idx_P1;

	if (indices==0x0)
	{
		// Compute metric for all pairs
		short i,j;
		if (idx_in>=num_dtrs*(num_dtrs-1)/2) return;
		// from ij, determine i and j i<j
		get_ij(idx_in,num_dtrs,i,j);
		// access data for the the i-th and jth projection
		idx_P0=i;
		idx_P1=j;
	}
	else
	{
		if (idx_in>=num_pairs) return;
		// Look up the first two indices and set Cs and Pinvs accordingly
		idx_P0=indices[4*idx_in+0];
		idx_P1=indices[4*idx_in+1];
	}
	// pinv(H*P)'=-inv(H)'*pinv(P)'
	computeK01(
		n_x2,
		n_y2,
		Cs+4*idx_P0,
		Cs+4*idx_P1,
		PinvTs+12*idx_P0,
		PinvTs+12*idx_P1,
		object_radius_mm,
		num_samples,
		dkappa,
		K01s+idx_in*16+0,
		K01s+idx_in*16+8
	);

}

/// Compute Epipolar line and convert to (angle, distance). Then sample pre-computed DTR. length=sqrt(line[0]*line[0]+line[1]*line[1]).
template <bool is_derivative>
__device__ float getRedundancy(const float *K, cudaTextureObject_t dtr, float range_t, float x0, float x1)
{
	// Find corresponding epipolar lines for plane at angle kappa (same as culaut::xgemm<float,3,2,1>(K,x_k,l);)
	float line[]={K[0]*x0+K[3]*x1,K[1]*x0+K[4]*x1,K[2]*x0+K[5]*x1};
	
	// Convert line to angle distance and compute texture coordinates to be sampled.
	bool angle_moved_by_pi=lineToSampleDtr(line,range_t);

	// Sample DTR and account for symmetry (up to sign in case of derivative)
	if (is_derivative)
		if (angle_moved_by_pi)
			return -tex2D<float>(dtr,line[0],line[1]);
	return +tex2D<float>(dtr,line[0],line[1]);
}

template <bool is_derivative>
__device__ inline float consistencyForPlusMinusKappa(
		float kappa,				//< Angle of epipolar plane
		const float K0[8],			//< Mapping from angle to line 0
		const float K1[8],			//< Mapping from angle to line 1
		cudaTextureObject_t dtr0,	//< Radon derivatives
		cudaTextureObject_t dtr1,	//< Radon derivatives
		float	range_t				//< Max. distance to center encoded in Radon space
	)
{
	// Get point on unit circle of two-space for projection to epipolar lines
	float x_kappa[2];
	__sincosf(kappa,x_kappa+1,x_kappa);

	// Compare redundant information for +kappa
	float vp=
		+getRedundancy<is_derivative>(K0,dtr0,range_t,x_kappa[0],x_kappa[1])
		-getRedundancy<is_derivative>(K1,dtr1,range_t,x_kappa[0],x_kappa[1]);

	// Compare redundant information for -kappa
	x_kappa[0]*=-1;
	float vm=
		+getRedundancy<is_derivative>(K0,dtr0,range_t,x_kappa[0],x_kappa[1])
		-getRedundancy<is_derivative>(K1,dtr1,range_t,x_kappa[0],x_kappa[1]);

	// Return consistency
	return (vp*vp + vm*vm)*K0[6];
}

template <bool is_derivative>
__device__ inline void consistencyForPlusMinusKappa_XCORR(
		float   kappa,				//< Angle of epipolar plane
		const float K0[8],			//< Mapping from angle to line 0
		const float K1[8],			//< Mapping from angle to line 1
		cudaTextureObject_t dtr0,	//< Radon derivatives
		cudaTextureObject_t dtr1,	//< Radon derivatives
		float range_t,				//< Max. distance to center encoded in Radon space
		float *xyxxyyxy,			//< Intermediate values for correlation computation
		float one_over_n			//< 1/n, where n is the number of samples (n=2*kappa_max/dkappa)
	)
{
	// Get point on unit circle of two-space for projection to epipolar lines
	float x_kappa[2];
	__sincosf(kappa,x_kappa+1,x_kappa);

	// Compare redundant information for +kappa

	float xp=getRedundancy<is_derivative>(K0,dtr0,range_t,x_kappa[0],x_kappa[1]);
	float yp=getRedundancy<is_derivative>(K1,dtr1,range_t,x_kappa[0],x_kappa[1]);

	// Compare redundant information for -kappa
	x_kappa[0]*=-1;

	float xm=getRedundancy<is_derivative>(K0,dtr0,range_t,x_kappa[0],x_kappa[1]);
	float ym=getRedundancy<is_derivative>(K1,dtr1,range_t,x_kappa[0],x_kappa[1]);

	// Accumulate intermediate values for computation of cerrelation coefficient
	atomicAdd(xyxxyyxy+0 , one_over_n*(xp   +xm   )); // 0 x
	atomicAdd(xyxxyyxy+1 , one_over_n*(yp   +ym   )); // 1 y
	atomicAdd(xyxxyyxy+2 , one_over_n*(xp*xp+xm*xm)); // 2 xx
	atomicAdd(xyxxyyxy+3 , one_over_n*(yp*yp+ym*ym)); // 3 yy
	atomicAdd(xyxxyyxy+4 , one_over_n*(xp*yp+xm*ym)); // 4 xy

}

template <bool is_derivative, bool use_corr>
__global__ void kernelEpipolarCosistency(
		const float *K01s,		  	//< Mapping from [cos(kappa) sin(kappa) to lines 0/1
		cudaTextureObject_t* dtrs,	//< DTRs as GPU textures
		float range_t,				//< Max. distance to center encoded in Radon space
		int num_pairs,				//< Explicit assigment of projection matrices and dtrs. Can be zero.
		const int *indices,			//< num_indices*4 (P0,P1,dtr0,dtr1). Can be null. If not provided, all n*(n-1)*2 pairs are computed.
		float *out,					//< num_indices result values, unless indices is null. Then nxn matrix. Only sub-diagonal elements are written to.
		float *out_corr				//< Intermediate values for correlation computation
	)
{
	// Find index of current thread
	int idx_x = blockIdx.x * blockDim.x + threadIdx.x;
	int idx_y = blockIdx.y * blockDim.y + threadIdx.y;
	if (idx_x>num_pairs) return;

#ifdef USE_SHARED_MEM_FOR_K01
	__shared__ float K0[8];
	__shared__ float K1[8];
	if (threadIdx.y<8)
	{
		K0[threadIdx.y]=K01s[threadIdx.y+idx_x*16];
		K1[threadIdx.y]=K01s[threadIdx.y+idx_x*16+8];
	}
	__syncthreads();
#else
	const float *K0=K01s+idx_x*16;
	const float *K1=K01s+idx_x*16+8;
#endif
	
	// Start summation and determine weight.
	if (idx_y==0)
	{
		out[idx_x]=0.f;
		// less than 9° -> zero and greater than 36° -> one. Smooth degree four polynomial in between.
		const float Pi=3.14159265359f;
		const float no_contrib=0.05f*Pi, full_contrib=0.2f*Pi;
		out_corr[use_corr?idx_x*6+5:idx_x]=1.0f; // K0[6]; // 1.0f-weighting((fabs(K0[7]-Pi)-no_contrib)/full_contrib);
	}

	// Current thread: Compute consistency for epipolar plane of angle +/-kappa
	float dkappa=K1[6];
	float kappa_max=K1[7];
	float kappa=dkappa*0.5f+dkappa*idx_y;

	// Check to see if out of range
	if (kappa>=kappa_max) return;
	
	// Compute consistency
	if (!use_corr)
	{
		float consistency=consistencyForPlusMinusKappa<is_derivative>(kappa,K0,K1,dtrs[indices[4*idx_x+2]],dtrs[indices[4*idx_x+3]],range_t);
		// We assume that the dtrs are zero only when the lines no longer intersect the image planes.
		atomicAdd(out+idx_x, consistency*dkappa);
	}
	else
	{
		float *xyxxyyxy=out_corr+idx_x*6;
		consistencyForPlusMinusKappa_XCORR<is_derivative>(kappa,K0,K1,dtrs[indices[4*idx_x+2]],dtrs[indices[4*idx_x+3]],range_t, xyxxyyxy, kappa_max/kappa);
	}
}

template <bool is_derivative, bool use_corr>
__global__ void kernelEpipolarCosistency(
		const float *K01s,		  	//< Mapping from [cos(kappa) sin(kappa) to lines 0/1
		int num_dtrs,				//< Explicit assigment of projection matrices and dtrs. Can be zero.
		cudaTextureObject_t* dtrs,	//< DTRs as GPU textures
		float range_t,				//< Max. distance to center encoded in Radon space
		float *out,					//< num_indices result values, unless indices is null. Then nxn matrix. Only sub-diagonal elements are written to.
		float *out_corr				//< Intermediate values for correlation computation
	)
{
	// Find index of current thread
	int idx_x = blockIdx.x * blockDim.x + threadIdx.x;
	int idx_y = blockIdx.y * blockDim.y + threadIdx.y;

	// Compute metric for all pairs
	short i,j;
	if (idx_x>=num_dtrs*(num_dtrs-1)/2) return;
	// from ij, determine i and j i<j
	get_ij(idx_x,num_dtrs,i,j);

#ifdef USE_SHARED_MEM_FOR_K01
	__shared__ float K0[8];
	__shared__ float K1[8];
	if (threadIdx.y<8)
	{
		K0[threadIdx.y]=K01s[threadIdx.y+idx_x*16];
		K1[threadIdx.y]=K01s[threadIdx.y+idx_x*16+8];
	}
	__syncthreads();
#else
	const float *K0=K01s+idx_x*16;
	const float *K1=K01s+idx_x*16+8;
#endif

	// Start summation
	if (idx_y==0)
	{
		out[i+j*num_dtrs]=0.f;
		// less than 18° -> zero and greater than 36° -> one. Smooth degree four polynomial in between.
		const float Pi=3.14159265359f;
		const float no_contrib=0.05f*Pi, full_contrib=0.2f*Pi;
		out_corr[use_corr?idx_x*6+5:idx_x]=1.0f; // 1.0f-weighting((fabs(K0[7]-Pi)-no_contrib)/full_contrib);
	}

	// Current thread: Compute consistency for epipolar plane of angle +/-kappa
	float dkappa=K1[6];
	float kappa_max=K1[7];
	float kappa=dkappa*0.5f+dkappa*idx_y;

	// Check to see if out of range
	if (kappa>=kappa_max) return;

	// Compute consistency
	if (!use_corr)
	{
		float consistency=consistencyForPlusMinusKappa<is_derivative>(kappa,K0,K1,dtrs[i],dtrs[j],range_t);
		atomicAdd(out+i+j*num_dtrs,consistency*dkappa);
	}
	else
	{
		float *xyxxyyxy=out_corr+idx_x*6;
		consistencyForPlusMinusKappa_XCORR<is_derivative>(kappa,K0,K1,dtrs[i],dtrs[j],range_t, xyxxyyxy, kappa_max/kappa);
	}
}

void epipolarConsistency(
	int n_x,				//< Width of original images in pixels
	int n_y,				//< Height of original images in pixels
	int num_dtrs,			//< Number of projection images (=# dtrs)
	char* dtrs_d,			//< DTRs as GPU textures cudaTextureObject_t
	int n_alpha,			//< Number of angular bins (y-direction)
	int n_t,				//< Number of line distance bins (x-direction)
	float step_alpha,		//< Angular spacing of radon transform
	float step_t,			//< Line distance spacing of radon transform
	int num_Ps,				//< Number of projection matrices
	float* Cs_d,			//< Source positions 4 * num_Ps, null spaces of projection matrices
	float* PinvTs_d,		//< Projection Matrix inverse transpose, column major 3x4 * num_Ps
	int num_pairs,			//< Explicit assigment of projection matrices and dtrs. Can be zero.
	int *indices_d,			//< num_indices*4 (P0,P1,dtr0,dtr1). Can be null. If not provided, all n*(n-1)*2 pairs are computed.
	float *K01s_d,			//< num_indices*16 for two 2*3 matrices mapping [cos(kappa sin(kappa)] to lines 0/1
	float *out_d,			//< num_indices result values, unless indices is null. Then nxn matrix. Only sub-diagonal elements are written to.
	float object_radius_mm,	//< Approximate size of object in mm (determines angular range, can be zero)
	float dkappa,			//< Angular distance between epipolar planes (can be zero if object_radius_mm and num_samples are given)
	bool isDerivative,		//< If false, normal Radon transform, else its derivative
	bool use_corr,			//< If null, SSD is used as a similarity measure, else correlation coeff is used.
	float *out_corr_d		//< Temporary memory for weights and NCC computation
	)
{
	cudaCheckState

	// Threads per block and problem size (one thread handles one pair.)
	dim3 block_size;
	block_size.x=32;
	// Make sure a single core does not do everything in case of small problems.
	if ((num_pairs==0 && num_dtrs<20) || (num_pairs!=0 && num_pairs<256))
		block_size.x=1;
	dim3 grid_size;
	if (indices_d == 0x0) num_pairs = num_dtrs*(num_dtrs - 1) / 2;
	
	grid_size.x = iDivUp(num_pairs,block_size.x);
	
	//std::cout << "Current config:\n";
	//std::cout << "block = " << block_size.x << " " << block_size.y << " " << block_size.z << std::endl;
	//std::cout << "grid  = " << grid_size.x << " " << grid_size.y << " " << grid_size.z << std::endl;
	//std::cout << "num_pairs = " << num_pairs << std::endl;

	// Number of pixels on the image diagonal
	float image_diagonal=n_t*step_t*2.f;

//	Utils::TimerWin32 time;
	// Start kernel (compute K01s)
	kernelEpipolarConsistencyComputeK01<<<grid_size, block_size>>>(
		n_x*0.5f,			//< Half Width of original images in pixels
		n_y*0.5f,			//< Half Height of original images in pixels
		num_dtrs,			//< Number of projection images (=# dtrs)
		num_Ps,				//< Number of projection matrices
		Cs_d,				//< Source positions 4 * num_Ps, null spaces of projection matrices
		PinvTs_d,			//< Projection Matrix inverse transpose, column major 3x4 * num_Ps
		num_pairs,			//< Explicit assigment of projection matrices and dtrs. Can be zero.
		indices_d,			//< num_indices*4 (P0,P1,dtr0,dtr1). Can be null. If not provided, all n*(n-1)*2 pairs are computed.
		K01s_d,	  			//< out: pre-computed mapping from 3D plane angles to 2D lines
		step_alpha,			//< Angular spacing of radon transform
		step_t,				//< Line distance spacing of radon transform
		object_radius_mm,	//< Approximate size of object in mm (determines angular range, can be zero)
		image_diagonal,		//< Number of samples in the order of pixels on the image diagonal
		dkappa				//< Angular distance between epipolar planes or zero for automatic
		);
	cudaDeviceSynchronize();
	cudaCheckState

	//double t_computeK01=time.getElapsedTime();

	// Distribute problem in a meaningful way: grid with #pairs in x and #angles/1024 in y direction, blocksize (1,1024)
	const float Pi=3.14159265359f;
	int max_num_samples=0;
	if (dkappa<=0.0)
		max_num_samples=(int)image_diagonal;
	else
		max_num_samples=(int)(Pi*0.5f/dkappa);
	block_size.x=4;
	block_size.y=256;
	if (indices_d)
		grid_size.x=iDivUp(num_pairs,block_size.x);
	else
		grid_size.x=iDivUp(num_dtrs*(num_dtrs-1)/2,block_size.x);
	grid_size.y=iDivUp(max_num_samples,block_size.y);

	//std::cout << "Current config:\n";
	//std::cout << "block = " << block_size.x << " " << block_size.y << " " << block_size.z << std::endl;
	//std::cout << "grid  = " << grid_size.x << " " << grid_size.y << " " << grid_size.z << std::endl;
	//std::cout << "num_pairs = " << num_pairs << std::endl;

	//std::cout << "Grid efficiency: " << max_angles << " / " << block_size.y*grid_size.y << std::endl;

	// Start kernel (sample Radon transforms)
	if (indices_d)
	{
		// todo ifs for template parameters
		if (use_corr)
		{
			if (isDerivative)
				kernelEpipolarCosistency<true,true><<<grid_size, block_size>>>( K01s_d, (cudaTextureObject_t *)dtrs_d, n_t*step_t, num_pairs, indices_d, out_d, out_corr_d );
			else
				kernelEpipolarCosistency<false,true><<<grid_size, block_size>>>( K01s_d, (cudaTextureObject_t *)dtrs_d, n_t*step_t, num_pairs, indices_d, out_d, out_corr_d );
		}
		else
		{
			if (isDerivative)
				kernelEpipolarCosistency<true,false><<<grid_size, block_size>>>( K01s_d, (cudaTextureObject_t *)dtrs_d, n_t*step_t, num_pairs, indices_d, out_d, out_corr_d );
			else
				kernelEpipolarCosistency<false,false><<<grid_size, block_size>>>( K01s_d, (cudaTextureObject_t *)dtrs_d, n_t*step_t, num_pairs, indices_d, out_d, out_corr_d );
		}
	}
	else
	{
		if (use_corr)
		{
			if (isDerivative)
				kernelEpipolarCosistency<true,true><<<grid_size, block_size>>>( K01s_d, num_dtrs, (cudaTextureObject_t *)dtrs_d, n_t*step_t, out_d, out_corr_d );
			else
				kernelEpipolarCosistency<false,true><<<grid_size, block_size>>>( K01s_d, num_dtrs, (cudaTextureObject_t *)dtrs_d, n_t*step_t, out_d, out_corr_d );
		}
		else
		{
			if (isDerivative)
				kernelEpipolarCosistency<true,false><<<grid_size, block_size>>>( K01s_d, num_dtrs, (cudaTextureObject_t *)dtrs_d, n_t*step_t, out_d, out_corr_d );
			else
				kernelEpipolarCosistency<false,false><<<grid_size, block_size>>>( K01s_d, num_dtrs, (cudaTextureObject_t *)dtrs_d, n_t*step_t, out_d, out_corr_d );
		}
	}
	cudaDeviceSynchronize();
	cudaCheckState
	// double t_computeECC=time.getElapsedTime();

	// std::cout << "time ECC: " << t_computeK01 <<" + "<< t_computeECC << " = " << time.getTotalTime() << std::endl;

}
