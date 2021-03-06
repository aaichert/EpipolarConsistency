#ifndef __cuda_texture_array_hxx

namespace UtilsCuda {

	/// Creates an array to be bound to a 3D texture unit
	template <typename T> struct TextureArray3D {
		cudaArray				*cu_array;
		cudaChannelFormatDesc	channelDesc;
		cudaExtent				size;
		TextureArray3D(T* host=0x0, int nx=0, int ny=0, int nz=0)
		{
			// Defaults to invalid instance
			if (!host) {
				size.width=size.height=size.depth=0;
				cu_array=0x0;
				return;
			}
			// Size of array
			size.width=nx;
			size.height=ny;
			size.depth=nz;
			// Create 3D array
			channelDesc = cudaCreateChannelDesc<T>();
			cudaMalloc3DArray(&cu_array, &channelDesc, size);
			cudaCheckState
			// Copy data to 3D array
			cudaMemcpy3DParms copyParams = {0};
			copyParams.srcPtr   = make_cudaPitchedPtr((void*)host, nx*sizeof(T), nx, ny);
			copyParams.dstArray = cu_array;
			copyParams.extent   = size;
			copyParams.kind     = cudaMemcpyHostToDevice;
			cudaMemcpy3D(&copyParams);
			cudaCheckState
		}

		/// expects a texture<float,3>
		template <typename cutex> void bind(cutex &tex) const
		{
			cudaBindTextureToArray(tex, cu_array, channelDesc);
			cudaCheckState
		}

	};

	/// Creates an array to be bound to a 2D texture unit
	template <typename T> struct TextureArray2D {
		cudaArray				*cu_array;
		cudaChannelFormatDesc	channelDesc;
		struct {
			int width;
			int height;
		} size;
		TextureArray2D(T* host=0x0, int n_x=0, int n_y=0)
		{
			size.width=n_x;size.height=n_y;
			// Defaults to invalid instance
			if (host) {
				cu_array=0x0;
				return;
			}
			cudaArray* cu_array;
			channelDesc = cudaCreateChannelDesc<T>();
			cudaMemcpyToArray(cu_array, 0, 0, host, n_x*n_y*sizeof(T), cudaMemcpyHostToDevice);
			cudaMallocArray(&cu_array, &channelDesc, n_x, n_y);
		}

		/// expects a texture<float,2>
		template <typename cutex> void bind(cutex &tex) const
		{
			cudaBindTextureToArray(tex, cu_array, channelDesc);
			cudaCheckState
		}

	};
	
	typedef TextureArray3D<float> Tex3Df;
	typedef TextureArray2D<float> Tex2Df;

} // namespace UtilsCuda

#endif // __cuda_texture_array_hxx
