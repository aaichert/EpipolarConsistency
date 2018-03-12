#include "UtilsCuda.hxx"

template <typename T, bool normalized_coords>
__global__ void kernel_sampleTex2D(cudaTextureObject_t input_image, int n_x, int n_y, T* output_d)
{
	// Find index of current thread
	int idx_x = blockIdx.x * blockDim.x + threadIdx.x;
	int idx_y = blockIdx.y * blockDim.y + threadIdx.y;
	if (idx_x>=n_x) return;
	if (idx_y>=n_y) return;
	int idx=idx_y*n_x+idx_x;
	// Sample
	if (normalized_coords)
		output_d[idx]=tex2D<T>(input_image,(idx_x+.5f)/n_x,(idx_y+.5f)/n_y);
	else
		output_d[idx]=tex2D<T>(input_image,idx_x+.5f,idx_y+.5f);
}

template <typename T, bool normalized_coords>
void cuda_sampleTex2D(cudaTextureObject_t input_image, int n_x, int n_y, T* output_d)
{
	dim3 block_size;
	block_size.x=32;
	block_size.y=32;
	dim3 grid_size;
	grid_size.x = iDivUp(n_x,block_size.x);
	grid_size.y = iDivUp(n_y,block_size.y);

	kernel_sampleTex2D<T,normalized_coords><<<grid_size, block_size>>>(input_image,  n_x, n_y, output_d);
	cudaDeviceSynchronize();
	cudaCheckState
}

template <typename T, bool normalized_coords>
__global__ void kernel_sampleTex3D(cudaTextureObject_t input_image, int n_x, int n_y, int n_z, T* output_d)
{
	// Find index of current thread
	int idx_x = blockIdx.x * blockDim.x + threadIdx.x;
	int idx_y = blockIdx.y * blockDim.y + threadIdx.y;
	int idx_z = blockIdx.z * blockDim.z + threadIdx.z;
	if (idx_x>=n_x) return;
	if (idx_y>=n_y) return;
	if (idx_z>=n_z) return;
	int idx=idx_z*n_x*n_y+idx_y*n_x+idx_x;
	// Sample
	if (normalized_coords)
		output_d[idx]=tex3D<T>(input_image,(idx_x+.5f)/n_x,(idx_y+.5f)/n_y,(idx_z+.5f)/n_z);
	else
		output_d[idx]=tex3D<T>(input_image,idx_x+.5f,idx_y+.5f,idx_z+.5f);
}

template <typename T, bool normalized_coords>
void cuda_sampleTex3D(cudaTextureObject_t input_image, int n_x, int n_y, int n_z, T* output_d)
{
	dim3 block_size;
	block_size.x=16;
	block_size.y=16;
	block_size.y=16;
	dim3 grid_size;
	grid_size.x = iDivUp(n_x,block_size.x);
	grid_size.y = iDivUp(n_y,block_size.y);
	grid_size.z = iDivUp(n_z,block_size.z);

	kernel_sampleTex3D<T,normalized_coords><<<grid_size, block_size>>>(input_image,  n_x, n_y, n_z, output_d);
	cudaDeviceSynchronize();
	cudaCheckState
}


template void cuda_sampleTex2D<float         , true>(cudaTextureObject_t input_image, int n_x, int n_y, float         * output_d);
template void cuda_sampleTex2D<float2        , true>(cudaTextureObject_t input_image, int n_x, int n_y, float2        * output_d);
template void cuda_sampleTex2D<float4        , true>(cudaTextureObject_t input_image, int n_x, int n_y, float4        * output_d);
template void cuda_sampleTex2D<unsigned char , true>(cudaTextureObject_t input_image, int n_x, int n_y, unsigned char * output_d);
template void cuda_sampleTex2D<uchar2        , true>(cudaTextureObject_t input_image, int n_x, int n_y, uchar2        * output_d);
template void cuda_sampleTex2D<uchar4        , true>(cudaTextureObject_t input_image, int n_x, int n_y, uchar4        * output_d);
template void cuda_sampleTex2D<short         , true>(cudaTextureObject_t input_image, int n_x, int n_y, short         * output_d);
template void cuda_sampleTex2D<unsigned short, true>(cudaTextureObject_t input_image, int n_x, int n_y, unsigned short* output_d);


template void cuda_sampleTex3D<float         , true>(cudaTextureObject_t input_image, int n_x, int n_y,int n_z, float         * output_d);
template void cuda_sampleTex3D<float2        , true>(cudaTextureObject_t input_image, int n_x, int n_y,int n_z, float2        * output_d);
template void cuda_sampleTex3D<float4        , true>(cudaTextureObject_t input_image, int n_x, int n_y,int n_z, float4        * output_d);
template void cuda_sampleTex3D<unsigned char , true>(cudaTextureObject_t input_image, int n_x, int n_y,int n_z, unsigned char * output_d);
template void cuda_sampleTex3D<uchar2        , true>(cudaTextureObject_t input_image, int n_x, int n_y,int n_z, uchar2        * output_d);
template void cuda_sampleTex3D<uchar4        , true>(cudaTextureObject_t input_image, int n_x, int n_y,int n_z, uchar4        * output_d);
template void cuda_sampleTex3D<short         , true>(cudaTextureObject_t input_image, int n_x, int n_y,int n_z, short         * output_d);
template void cuda_sampleTex3D<unsigned short, true>(cudaTextureObject_t input_image, int n_x, int n_y,int n_z, unsigned short* output_d);


template void cuda_sampleTex2D<float         , false>(cudaTextureObject_t input_image, int n_x, int n_y, float         * output_d);
template void cuda_sampleTex2D<float2        , false>(cudaTextureObject_t input_image, int n_x, int n_y, float2        * output_d);
template void cuda_sampleTex2D<float4        , false>(cudaTextureObject_t input_image, int n_x, int n_y, float4        * output_d);
template void cuda_sampleTex2D<unsigned char , false>(cudaTextureObject_t input_image, int n_x, int n_y, unsigned char * output_d);
template void cuda_sampleTex2D<uchar2        , false>(cudaTextureObject_t input_image, int n_x, int n_y, uchar2        * output_d);
template void cuda_sampleTex2D<uchar4        , false>(cudaTextureObject_t input_image, int n_x, int n_y, uchar4        * output_d);
template void cuda_sampleTex2D<short         , false>(cudaTextureObject_t input_image, int n_x, int n_y, short         * output_d);
template void cuda_sampleTex2D<unsigned short, false>(cudaTextureObject_t input_image, int n_x, int n_y, unsigned short* output_d);


template void cuda_sampleTex3D<float         , false>(cudaTextureObject_t input_image, int n_x, int n_y,int n_z, float         * output_d);
template void cuda_sampleTex3D<float2        , false>(cudaTextureObject_t input_image, int n_x, int n_y,int n_z, float2        * output_d);
template void cuda_sampleTex3D<float4        , false>(cudaTextureObject_t input_image, int n_x, int n_y,int n_z, float4        * output_d);
template void cuda_sampleTex3D<unsigned char , false>(cudaTextureObject_t input_image, int n_x, int n_y,int n_z, unsigned char * output_d);
template void cuda_sampleTex3D<uchar2        , false>(cudaTextureObject_t input_image, int n_x, int n_y,int n_z, uchar2        * output_d);
template void cuda_sampleTex3D<uchar4        , false>(cudaTextureObject_t input_image, int n_x, int n_y,int n_z, uchar4        * output_d);
template void cuda_sampleTex3D<short         , false>(cudaTextureObject_t input_image, int n_x, int n_y,int n_z, short         * output_d);
template void cuda_sampleTex3D<unsigned short, false>(cudaTextureObject_t input_image, int n_x, int n_y,int n_z, unsigned short* output_d);














