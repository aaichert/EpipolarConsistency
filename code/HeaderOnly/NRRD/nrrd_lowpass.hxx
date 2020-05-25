// Created by A. Aichert on Fri Feb 26th 2016
// WORK IN PROGRESS
// added 1D convolution for x and y dimension by Lina Felsner on Tue 21.3.17

#ifndef __NRRD_CONVOLVE_HXX
#define __NRRD_CONVOLVE_HXX

#include "nrrd_image.hxx"

namespace NRRD
{
	/// A Gaussian curve.
	inline double gaussian(double x, double sigma)
	{
		return exp(-0.5*pow(x/sigma,2));
	}
		
	/// Create a 1D Gaussian Kernel of size 2*k+1. Default values are for downsampling by factor of 2.
	inline std::vector<double>  gaussianKernel(double sigma=1.84,int k=5)
	{
		int n=k*2+1;
		std::vector<double> kernel(n);
		double sum=0;
		for (int x=-k;x<=k;x++)
		{
			double v=gaussian(x,sigma);
			sum+=v;
			kernel[x+k]=v;
		}
		for (int i=0;i<n;i++)
			kernel[i]/=sum;
		return kernel;
	}

	/// Clamp value to [0 max] range
	template <typename T>
	T clamp(const T &x, const T& max)
	{
		if (x<0) return 0;
		if (x>max) return max;
		return x;
	}
	#pragma optimize( "g", on )
	/// 2D Convolution for separable kernels. Result written to "img". Convolves z-slices separately. "work" must be a 2D image with same x/y size as img.
	template <typename T, typename Kernel>	
	bool convolve2D(NRRD::ImageView<T> &img, int kx, int ky, Kernel kernelx, Kernel kernely, NRRD::ImageView<T> &work)
	{
		int w=img.size(0);
		int h=img.size(1);
		if (work.size(0)!=w || work.size(1)!=h)
			return false;
		for (int y=0;y<h;y++)
			#pragma omp parallel for
			for (int x=0;x<w;x++)
			{
				double sum=0;
				for (int o=-kx;o<kx;o++)
				{
					int xo=clamp(x+o,w-1);
					sum+=img(xo,y)*kernelx[o+kx];
				}
				work.pixel(x,y)=(T)sum;
			}
		for (int x=0;x<w;x++)
			#pragma omp parallel for
			for (int y=0;y<h;y++)
			{
				double sum=0;
				for (int o=-ky;o<ky;o++)
				{
					int yo=clamp(y+o,h-1);
					sum+=work(x,yo)*kernelx[o+ky];
				}
				img.pixel(x,y)=(T)sum;
			}
		return true;
	}

	/// 3D Convolution for separable kernels. Result written to "img". Work must be same size as input.
	template <typename T, typename Kernel>	
	bool convolve3D(NRRD::ImageView<T> &img, int kx, int ky, int kz, Kernel kernelx, Kernel kernely, Kernel kernelz, NRRD::ImageView<T> &work)
	{
		if (img.dimension()<3 || work.dimension()<3) return false;
		int w=img.size(0);
		int h=img.size(1);
		int d=img.size(2);
		if (work.size(0)!=w || work.size(1)!=h || work.size(2)!=d)
			return false;
		convolveX(img ,kx,kernelx,work);
		convolveY(work,ky,kernely,img);
		convolveZ(img ,kz,kernelz,work);
		img.copyDataFrom((T*)work);
		return true; 
	}


	/// 1D Convolution along X. Result written to "out".
	template <typename T, typename Kernel>
	bool convolveX(const NRRD::ImageView<T> &img, int kx, Kernel kernelx, NRRD::ImageView<T> &out)
	{
		int w = img.size(0);
		int h = img.size(1);
		int d = img.size(2);
		if (out.size(0) != w || out.size(1) != h || out.size(2) != d)
			return false;

		for (int z = 0; z<d; z++)
		{
#pragma omp parallel for
			for (int y = 0; y < h; y++)
			{
				for (int x = 0; x < w; x++)
				{
					double sum = 0;
					for (int o = -kx; o < kx; o++)
					{
						int xo = clamp(x + o, w - 1);
						sum += img(xo, y, z)*kernelx[o + kx];
					}
					out.pixel(x, y, z) = (T)sum;
				}
			}
		}
	}

	/// 1D Convolution along Y. Result written to "out".
	template <typename T, typename Kernel>
	bool convolveY(const NRRD::ImageView<T> &img, int ky, Kernel kernely, NRRD::ImageView<T> &out)
	{
		int w = img.size(0);
		int h = img.size(1);
		int d = img.size(2);
		if (out.size(0) != w || out.size(1) != h || out.size(2) != d)
			return false;
		for (int z = 0; z < d; z++)
		{
#pragma omp parallel for
			for (int x = 0; x < w; x++)
			{
				for (int y = 0; y < h; y++)
				{
					double sum = 0;
					for (int o = -ky; o < ky; o++)
					{
						int yo = clamp(y + o, h - 1);
						sum += img(x, yo, z)*kernely[o + ky];
					}
					out.pixel(x, y, z) = (T)sum;
				}
			}
		}
	}
	
	/// 1D Convolution along Z. Result written to "out".
	template <typename T, typename Kernel>
	bool convolveZ(const NRRD::ImageView<T> &img, int kz, Kernel kernelz, NRRD::ImageView<T> &out)
	{
		int w = img.size(0);
		int h = img.size(1);
		int d = img.size(2);
		if (out.size(0) != w || out.size(1) != h || out.size(2) != d)
			return false;
		for (int z = 0; z < d; z++)
		{
			for (int x = 0; x < w; x++)
			{
#pragma omp parallel for
				for (int y = 0; y < h; y++)
				{
					double sum = 0;
					for (int o = -kz; o < kz; o++)
					{
						int zo = clamp(z + o, d - 1);
						sum += img(x, y, zo)*kernelz[o + kz];
					}
					out.pixel(x, y, z) = (T)sum;
				}
			}
		}
	}

	// Lowpass image filter. Default parameters correspnd to a reduction of image size by 2.
	template <typename T>
	void lowpass2D(NRRD::ImageView<T> &img, double sigma=1.84, int k=5)
	{
		NRRD::Image<T> work(img.size(0),img.size(1));
		auto kernel=NRRD::gaussianKernel(sigma, k);
		NRRD::convolve2D(img,k,k,kernel.data(),kernel.data(),work);
	}

	// Lowpass image filter. Default parameters correspnd to a reduction of image size by 2.
	template <typename T>
	void lowpass3D(NRRD::ImageView<T> &img, double sigma=1.84, int k=5)
	{
		NRRD::Image<T> work(img.size(0),img.size(1),img.size(2));
		auto kernel=NRRD::gaussianKernel(sigma, k);
		NRRD::convolve3D(img,k,k,k,kernel.data(),kernel.data(),kernel.data(),work);
	}



} // namespace NRRD

#endif // __NRRD_CONVOLVE_HXX
