#include "UtilsCuda.hxx"
#include "CudaMemory.h"

// 2do at least the kernel should be in shared mem

#include <vector>

// 1D convolution along rows of a single-channel 2D image
__global__ void kernel_image2D1C_ConvolveRow(float* img, int n_x, int n_y, short k, float *kernel, float* out)
{
	// Find index of current thread
	int idx_x = blockIdx.x * blockDim.x + threadIdx.x;
	int idx_y = blockIdx.y * blockDim.y + threadIdx.y;
	if (idx_x>=n_x) return;
	if (idx_y>=n_y) return;

	float sum=0;
	for (short i=-k;i<=k;i++)
	{
		short x=idx_x+i;
		if (x<0) x=0;
		if (x>=n_x) x=n_x-1;
		sum+=kernel[i+k]*img[idx_y*n_x+x];
	}
	out[idx_y*n_x+idx_x]=sum;
}

void _image2D1C_ConvolveRow(float* img, int n_x, int n_y, short k,   float *kernel, float* out)
{
	dim3 block_size;
	block_size.x=32;
	block_size.y=32;
	dim3 grid_size;
	grid_size.x = iDivUp(n_x,block_size.x);
	grid_size.y = iDivUp(n_y,block_size.y);

	cudaCheckState
	kernel_image2D1C_ConvolveRow<<<grid_size, block_size>>>(img, n_x, n_y, k, kernel, out);
	cudaDeviceSynchronize();
	cudaCheckState
}

// 1D convolution along columns of a single-channel 2D image
__global__ void kernel_image2D1C_ConvolveColumn(float* img, int n_x, int n_y, short k, float *kernel, float* out)
{
	// Find index of current thread
	int idx_x = blockIdx.x * blockDim.x + threadIdx.x;
	int idx_y = blockIdx.y * blockDim.y + threadIdx.y;
	if (idx_x>=n_x) return;
	if (idx_y>=n_y) return;
	float sum=0;
	for (short i=-k;i<=k;i++)
	{
		short y=idx_y+i;
		if (y<0) y=0;
		if (y>=n_y) y=n_y-1;
		sum+=kernel[i+k]*img[y*n_x+idx_x];
	}
	out[idx_y*n_x+idx_x]=sum;
}

void _image2D1C_ConvolveColumn(float* img, int n_x, int n_y, short k, float *kernel, float* out)
{
	dim3 block_size;
	block_size.x=32;
	block_size.y=32;
	dim3 grid_size;
	grid_size.x = iDivUp(n_x,block_size.x);
	grid_size.y = iDivUp(n_y,block_size.y);

	cudaCheckState
	kernel_image2D1C_ConvolveColumn<<<grid_size, block_size>>>(img, n_x, n_y, k, kernel, out);
	cudaDeviceSynchronize();
	cudaCheckState
}

void _image2D1C_ConvolveSperable(float* img, int n_x, int n_y, short kx, float *kernelx, short ky, float *kernely, float* tmp)
{
	_image2D1C_ConvolveRow(img, n_x,n_y, kx,kernelx, tmp);
	_image2D1C_ConvolveColumn(tmp, n_x,n_y, ky,kernely, img);
}

/// A Gaussian curve.
inline float gaussian(float x, float sigma)
{
	return expf(-0.5f*powf(x/sigma,2));
}

void _image2D1C_Gaussian(float* img, int n_x, int n_y, short k, float sigma, float* tmp)
{
	int n=k*2+1;
	std::vector<float> kernel(n);
	float sum=0;
	for (int x=-k;x<=k;x++)
		sum+=(kernel[x+k]=gaussian((float)x,sigma));
	for (int i=0;i<n;i++)
		kernel[i]/=sum;
	UtilsCuda::MemoryBlock<float> kernel_device(n,&kernel[0]);
	UtilsCuda::MemoryBlock<float> tmp_mem; // allocate if needed
	if (!tmp) tmp_mem.allocate(n_x*n_y);
	if (!tmp) tmp=tmp_mem;
	_image2D1C_ConvolveRow(img, n_x,n_y, k,kernel_device, tmp);
	_image2D1C_ConvolveColumn(tmp, n_x,n_y, k,kernel_device, img);
}
