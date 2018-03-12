#include "CudaAlgorithms.h"



extern void _image2D1C_ConvolveRow(float* img, int n_x, int n_y, short k, float *kernel, float* out);


extern void _image2D1C_ConvolveColumn(float* img, int n_x, int n_y, short k, float *kernel, float* out);


extern void _image2D1C_ConvolveSperable(float* img, int n_x, int n_y, short kx, float *kernelx, short ky, float *kernely, float* tmp);


extern void _image2D1C_Gaussian(float* img, int n_x, int n_y, short k, float sigma, float* tmp);


namespace UtilsCuda
{

	void image2D1C_ConvolveRow(float* img, int n_x, int n_y, short k, float *kernel, float* out)
	{
		_image2D1C_ConvolveRow( img, n_x, n_y, k, kernel, out);
	}

	void image2D1C_ConvolveColumn(float* img, int n_x, int n_y, short k, float *kernel, float* out)
	{
		_image2D1C_ConvolveColumn(img, n_x, n_y, k, kernel, out);
	}


	void image2D1C_ConvolveSperable(float* img, int n_x, int n_y, short kx, float *kernelx, short ky, float *kernely, float* tmp)
	{
		_image2D1C_ConvolveSperable( img, n_x, n_y, kx, kernelx, ky, kernely, tmp);
	}


	void image2D1C_Gaussian(float* img, int n_x, int n_y, short k, float sigma, float* tmp)
	{
		_image2D1C_Gaussian( img, n_x, n_y, k, sigma, tmp);
	}

} // namespace UtilsCuda
