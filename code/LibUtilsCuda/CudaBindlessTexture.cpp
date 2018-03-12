#include "CudaBindlessTexture.h"

#include <channel_descriptor.h>
#include <cstring>

using std::memset;

template <typename T, bool is_normalized>
extern void cuda_sampleTex2D(cudaTextureObject_t input_image, int n_x, int n_y, T* output_d);

template <typename T, bool is_normalized>
extern void cuda_sampleTex3D(cudaTextureObject_t input_image, int n_x, int n_y, int n_z, T* output_d);

namespace UtilsCuda
{
	template <typename Type>
	BindlessTexture2D<Type>::BindlessTexture2D(int w, int h, const Type * buffer, bool buffer_is_device, bool interpolate, bool normalizedCoords)
		: normalizedCoords(normalizedCoords)
	{
		cudaCheckState
		size[0]=w;
		size[1]=h;
		// Allocate CUDA array in device memory
		cudaChannelFormatDesc channelDesc=cudaCreateChannelDesc<Type>();// cudaCreateChannelDesc(32,0,0,0,cudaChannelFormatKindFloat);
		cudaMallocArray(&array, &channelDesc, w, h);
		cudaCheckState
		cudaMemcpyToArray(array, 0, 0, buffer, sizeof(Type)*w*h, buffer_is_device ? cudaMemcpyDeviceToDevice : cudaMemcpyHostToDevice);
		cudaCheckState
		cudaResourceDesc resDescr;
		memset(&resDescr,0,sizeof(cudaResourceDesc));
		resDescr.resType = cudaResourceTypeArray;
		resDescr.res.array.array = array;

		cudaTextureDesc texDescr;
		memset(&texDescr,0,sizeof(cudaTextureDesc));
		texDescr.normalizedCoords = normalizedCoords;
		texDescr.filterMode  = interpolate?cudaFilterModeLinear:cudaFilterModePoint;
		texDescr.addressMode[0] = cudaAddressModeClamp;
		texDescr.addressMode[1] = cudaAddressModeClamp;
		texDescr.readMode =	cudaReadModeElementType;

		cudaCreateTextureObject(&tex, &resDescr, &texDescr, NULL);
		cudaCheckState
	}

	template <typename Type>
	BindlessTexture2D<Type>::~BindlessTexture2D()
	{
		cudaFreeArray(array);
		cudaDestroyTextureObject(tex);
		cudaCheckState
	}

	template <typename Type>
	void BindlessTexture2D<Type>::readback(MemoryBlock<Type>& buffer)
	{
		int n=size[0]*size[1];
		if (!n) {
			buffer.dealloc();
			return;
		}
		buffer.allocate(n);
		if (normalizedCoords)
			cuda_sampleTex2D<Type,true>(*this,size[0],size[1],buffer);
		else
			cuda_sampleTex2D<Type,true>(*this,size[0],size[1],buffer);
	}

	template <typename Type>
	BindlessTexture3D<Type>::BindlessTexture3D(int w, int h, int d, const Type * buffer, bool buffer_is_device, bool interpolate, bool normalizedCoords)
		: normalizedCoords(normalizedCoords)
	{
		size[0]=w;
		size[1]=h;
		size[2]=d;

		// Allocate CUDA array in device memory
		cudaChannelFormatDesc channelDesc=cudaCreateChannelDesc<Type>();
		cudaMalloc3DArray(&array, &channelDesc, make_cudaExtent(w,h,d), 0);
		cudaCheckState

		// cudaMemcpyToArray(array, 0, 0, buffer, sizeof(Type)*w*h*d, buffer_is_device ? cudaMemcpyDeviceToDevice : cudaMemcpyHostToDevice); // ?
		cudaMemcpy3DParms copyParams={0};
		copyParams.srcPtr=make_cudaPitchedPtr((void*)buffer, w*sizeof(Type), w, h);
		copyParams.dstArray=array;
		copyParams.extent=make_cudaExtent(w,h,d);
		copyParams.kind=cudaMemcpyHostToDevice;
		cudaMemcpy3D(&copyParams);
		cudaCheckState

		cudaResourceDesc resDescr;
		memset(&resDescr,0,sizeof(cudaResourceDesc));
		resDescr.resType = cudaResourceTypeArray;
		resDescr.res.array.array = array;

		cudaTextureDesc texDescr;
		memset(&texDescr,0,sizeof(cudaTextureDesc));
		texDescr.normalizedCoords = normalizedCoords;
		texDescr.filterMode  = interpolate?cudaFilterModeLinear:cudaFilterModePoint;
		texDescr.addressMode[0] = cudaAddressModeClamp;
		texDescr.addressMode[1] = cudaAddressModeClamp;
		texDescr.addressMode[2] = cudaAddressModeClamp;
		texDescr.readMode =	cudaReadModeElementType;

		cudaCreateTextureObject(&tex, &resDescr, &texDescr, NULL);
		cudaCheckState
	}

	template <typename Type>
	BindlessTexture3D<Type>::~BindlessTexture3D()
	{
		cudaFreeArray(array);
		cudaDestroyTextureObject(tex);
		cudaCheckState
	}

	template <typename Type>
	void BindlessTexture3D<Type>::readback(MemoryBlock<Type>& buffer)
	{
		int n=size[0]*size[1]*size[2];
		if (!n) {
			buffer.dealloc();
			return;
		}
		buffer.allocate(n);
		if (normalizedCoords)
			cuda_sampleTex3D<Type,true>(*this,size[0],size[1],size[2],buffer);
		else
			cuda_sampleTex3D<Type,false>(*this,size[0],size[1],size[2],buffer);
	}


	template class BindlessTexture2D<float>;
	template class BindlessTexture2D<float2>;
	template class BindlessTexture2D<float4>;

	template class BindlessTexture2D<unsigned char>;
	template class BindlessTexture2D<uchar2>;
	template class BindlessTexture2D<uchar4>;

	template class BindlessTexture2D<short>;
	template class BindlessTexture2D<unsigned short>;


	template class BindlessTexture3D<float>;
	template class BindlessTexture3D<float2>;
	template class BindlessTexture3D<float4>;

	template class BindlessTexture3D<unsigned char>;
	template class BindlessTexture3D<uchar2>;
	template class BindlessTexture3D<uchar4>;

	template class BindlessTexture3D<short>;
	template class BindlessTexture3D<unsigned short>;

} // namespace UtilsCuda

