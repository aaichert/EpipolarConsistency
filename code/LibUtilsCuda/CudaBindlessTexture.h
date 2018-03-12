#ifndef __cuda_bindless_texture_h
#define __cuda_bindless_texture_h

#include "UtilsCuda.hxx"

#include "CudaMemory.h"

// TODO make size a protected member and provide getter.

namespace UtilsCuda
{
	/// Bindless CUDA 2D Texture of Type (e.g. float, float2 unit4 etc.)
	/// buffer_is_device : if true, then the buffer is assumed to be a device pointer already
	/// interpolate		 : bilinear interpolation if true
	/// normalizedAccess : Access as texture coordinates rather than array index
	/// Type             : float float2 float4 unsigned char uchar2 uchar4 short unsigned short
	template <typename Type>
	class BindlessTexture2D
	{
	public:
		const bool normalizedCoords;
		int			size[2];
		cudaArray_t			array;
		cudaTextureObject_t tex;

		BindlessTexture2D(int w, int h, const Type * buffer, bool buffer_is_device=false, bool interpolate=true, bool normalizedCoords=false);

		~BindlessTexture2D();

		void readback(MemoryBlock<Type>& buffer);

		inline operator cudaTextureObject_t () const {return tex;}
	};

	/// Bindless CUDA 3D Texture of Type (e.g. float, float2 unit4 etc.)
	/// buffer_is_device : if true, then the buffer is assumed to be a device pointer already
	/// interpolate		 : bilinear interpolation if true
	/// normalizedAccess : Access as texture coordinates rather than array index
	/// Type             : float float2 float4 unsigned char uchar2 uchar4 short unsigned short
	template <typename Type>
	class BindlessTexture3D
	{
	public:
		const bool normalizedCoords;
		int			size[3];
		cudaArray_t			array;
		cudaTextureObject_t tex;

		BindlessTexture3D(int w, int h, int d, const Type * buffer, bool buffer_is_device=false, bool interpolate=true, bool normalizedCoords=false);

		~BindlessTexture3D();

		void readback(MemoryBlock<Type>& buffer);

		inline operator cudaTextureObject_t () const {return tex;}
	};

} // namespace UtilsCuda

#endif // __cuda_bindless_texture_h
