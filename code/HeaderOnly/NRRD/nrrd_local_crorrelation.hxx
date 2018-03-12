// Created by A. Aichert on Mon April 28th 2014
#ifndef __nrrd_local_correlation_hxx
#define __nrrd_local_correlation_hxx

#include "nrrd_image_view.hxx"

#include <cmath>

namespace NRRD {

/// return absolute value of local cross correlation
inline double xcorr(double suma, double sumb, double sumaa, double sumbb, double sumab, int n)
{
	if (n<=0) return -1;
	double nom=n*sumab - suma*sumb;
	double denom=(n*sumaa-suma*suma)*(n*sumbb-sumb*sumb);
	return std::abs(nom/sqrt(denom));
}

/// Compute cross correlation of two arbitrary images
template <typename T, typename TMask>
double computeCorrelation(
	const NRRD::ImageView<T>& A,
	const NRRD::ImageView<T>& B,
	int half_patch_size,
	const NRRD::ImageView<TMask>& mask,
	TMask active_label)
{
	double _sa=0,_sb=0 ,_saa=0 ,_sbb=0 ,_sab=0;
	int n=0;
	// #pragma omp 
	// sum reduction
	int l=A.length();
	for (int i=0;i<l;i++)
		if (mask[i]==active_label)
		{
			double a=A[i];
			double b=B[i];
			_sa+=a;
			_sb+=b;
			_saa+=a*a;
			_sbb+=b*b;
			_sab+=a*b;
			n++;
		}
		std::cout
			<< _sa  <<std::endl
			<< _sb  <<std::endl
			<< _saa <<std::endl
			<< _sbb <<std::endl
			<< _sab <<std::endl
			<< n    <<std::endl;
	return xcorr(_sa,_sb,_saa,_sbb,_sab,n);
}


/// Compute 3D cross correlation on local patches the slow way
template <typename T, typename TMask>
double computeLocalCorrelation(
	const NRRD::ImageView<T>& A,
	const NRRD::ImageView<T>& B,
	int half_patch_size,
	const NRRD::ImageView<TMask>& mask,
	TMask active_label,
	NRRD::ImageView<float>& out)
{
	// 2do assert dimension and equal size

	// Image size
	int w=A.size(0);
	int h=A.size(1);
	int d=A.dimension()>2?A.size(2):1;
	
	for (int z=0;z<d;z++)
	{
		for (int y=0;y<h;y++)
		{
			#pragma omp parallel for
			for (int x=0;x<w;x++)
			{
				if (mask(x,y,z)!=active_label) {
					out.pixel(x,y,z)=-1;
					continue;
				}
				double _sa=0,_sb=0 ,_saa=0 ,_sbb=0 ,_sab=0;
				int n=0;
				for (int dz=-half_patch_size;dz<=half_patch_size;dz++)
					for (int dy=-half_patch_size;dy<=half_patch_size;dy++)
						for (int dx=-half_patch_size;dx<=half_patch_size;dx++)
						{
							int ix=x+dx;
							int iy=y+dy;
							int iz=z+dz;
							if (ix>=0 && ix<w
								&& iy>=0 && iy<h
								&& iz>=0 && iz<d
								&& mask(ix,iy,iz)==active_label)
							{
								double a=A(x,y,0);
								double b=B(x,y,0);
								_sa+=a;
								_sb+=b;
								_saa+=a*a;
								_sbb+=b*b;
								_sab+=a*b;
								n++;
							}						
						}
				out.pixel(x,y,z)=xcorr(_sa,_sb,_saa,_sbb,_sab,n);
			}
		}
		std::cout << z+1 << " of " << d << std::endl;
	}
	// Return mean local correlation
	double mean=0;
	int l=out.length();
	int n=0;
	for (int i=0;i<l;i++)
		if (mask[i]==active_label) {
			mean+=out[l];
			n++;
		}
	return mean/n;
}

/// Compute 2D cross correlation on local patches for every z-slice
template <typename T, typename TMask>
double computeLocalCorrelation2D(
	const NRRD::ImageView<T>& A,
	const NRRD::ImageView<T>& B,
	int half_patch_size,
	const NRRD::ImageView<TMask>& mask,
	TMask active_label,
	NRRD::ImageView<float>& out)
{
	// Assert dimension and equal size
	if (A.dimension()!=B.dimension()||A.size(0)!=B.size(0)||A.size(1)!=B.size(1))
		return -1;
	// Image size
	int w=A.size(0);
	int h=A.size(1);
	int d=A.dimension()>2?A.size(2):1;
			
	// For every z-slice compute 2D sums
	for (int z=0;z<d;z++)
	{
		// For every voxel, sum over patch and compute correlation
		#pragma omp parallel for
		for (int y=0;y<h;y++)
		{
			// Initialize sums for recursion on x-line
			double _sa=0,_sb=0,_saa=0,_sbb=0,_sab=0;
			int n=0;
			// Compute sum in patch on border of image
			for (int dy=-half_patch_size;dy<=half_patch_size;dy++)
				for (int ix=0;ix<half_patch_size;ix++)
				{
					int iy=y+half_patch_size;
					if (iy>=0 && iy<h && mask(ix,iy,0)==active_label)
					{
						double a=A(ix,iy,0);
						double b=B(ix,iy,0);
						_sa+=a;
						_sb+=b;
						_saa+=a*a;
						_sbb+=b*b;
						_sab+=a*b;
						n++;
					}
				}
			out.pixel(0,y,z)=xcorr(_sa,_sb,_saa,_sbb,_sab,n);

			// Recursive summation over patches in x-direction			
			for (int x=1;x<w;x++)
			{
				for (int dy=-half_patch_size;dy<=half_patch_size;dy++)
				{
					// add top
					int ix=x+half_patch_size;
					int iy=y+dy;
					if (ix>0&&iy>0&&ix<w&&iy<h // in image bounds
						&& mask(x,y,z)==active_label)
						{
							double a=A(x,y,0);
							double b=B(x,y,0);
							_sa+=a;
							_sb+=b;
							_saa+=a*a;
							_sbb+=b*b;
							_sab+=a*b;
							n++;					
						} 
					// subtract bottom
					ix=x-half_patch_size;
					iy=y+dy;
					if (ix>0&&iy>0&&ix<w&&iy<h // in image bounds
						&& mask(x,y,z)==active_label)
						{
							double a=A(x,y,0);
							double b=B(x,y,0);
							_sa-=a;
							_sb-=b;
							_saa-=a*a;
							_sbb-=b*b;
							_sab-=a*b;
							n--;
						}					
				}
				out.pixel(x,y,z)=xcorr(_sa,_sb,_saa,_sbb,_sab,n);			
			}
		}
	}
	
	// Return mean local correlation
	double mean=0;
	int l=out.length();
	int n=0;
	for (int i=0;i<l;i++)
		if (mask[i]==active_label) {
			mean+=out[l];
			n++;
		}
	return mean/n;
}

} // namespace NRRD

#endif // __nrrd_local_correlation_hxx
