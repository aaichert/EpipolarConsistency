#ifndef __cuda_algorithms_h
#define __cuda_algorithms_h

namespace UtilsCuda
{

	// 1D convolution along rows of a single-channel 2D image. All pointer are in GPU memory.
	void image2D1C_ConvolveRow(float* img, int n_x, int n_y, short k, float *kernel, float* out);
	// 1D convolution along columns of a single-channel 2D image. All pointer are in GPU memory.
	void image2D1C_ConvolveColumn(float* img, int n_x, int n_y, short k, float *kernel, float* out);

	// 2D convolution of a single-channel 2D image with seperable kernel. All pointer are in GPU memory.
	void image2D1C_ConvolveSperable(float* img, int n_x, int n_y, short kx, float *kernelx, short ky, float *kernely, float* tmp);

	// Gaussian convolution of a single-channel 2D image. All pointer are in GPU memory.
	void image2D1C_Gaussian(float* img, int n_x, int n_y, short k, float sigma, float* tmp=0x0);

} // namespace UtilsCuda

#endif // __cuda_algorithms_h
