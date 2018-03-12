// Created by A. Aichert on Wed April 16th 2014

#ifndef __nrrd_morphology_hxx
#define __nrrd_morphology_hxx

#include "nrrd_image_view.hxx"

namespace NRRD {

	/// An int tripel
	struct Index3 {
		Index3(int _x, int _y, int _z) : x(_x), y(_y), z(_z) {}
		int x,y,z;
	};

	inline std::vector<Index3> sphere(int n)
	{
		std::vector<Index3> ret;
		for (int z=-n;z<=n;z++)
			for (int y=-n;y<=n;y++)
				for (int x=-n;x<=n;x++)
					if (std::sqrt((double)x*x+y*y+z*z)<=n)
						ret.push_back(Index3(x,y,z));
		return ret;
	}

	/// 1 voxel dilation
	template <typename T>
	void dilate3D(NRRD::ImageView<T> img, int radius=1)
	{
		std::vector<Index3>& structure_element=sphere(radius);
		if (!img) return;
		NRRD::Image<T> tmp;
		int nx=img.size(0);
		int ny=img.size(1);
		int nz=img.size(2);
		tmp.set(nx,ny,nz);
		for (int i=0;i<nx*ny*nz;i++)
			tmp[i]=img[i];
		for (int z=radius;z<nz-radius;z++)
			for (int y=radius;y<ny-radius;y++)
				#pragma omp parallel for
				for (int x=radius;x<nx-radius;x++)
				{
					T max=img(x,y,z);
					for (auto it=structure_element.begin();it!=structure_element.end();++it)
						if (max<tmp(x+it->x,y+it->y,z+it->z))
							max=tmp(x+it->x,y+it->y,z+it->z);
					img.pixel(x,y,z)=max;
				}
	}

	/// 1 voxel erosion
	template <typename T>
	void erode3D(NRRD::ImageView<T> img, int radius=1)
	{
		std::vector<Index3>& structure_element=sphere(radius);
		if (!img) return;
		NRRD::Image<T> tmp;
		int nx=img.size(0);
		int ny=img.size(1);
		int nz=img.size(2);
		tmp.set(nx,ny,nz);
		for (int i=0;i<nx*ny*nz;i++)
			tmp[i]=img[i];
		for (int z=radius;z<nz-radius;z++)
			for (int y=radius;y<ny-radius;y++)
				#pragma omp parallel for
				for (int x=radius;x<nx-radius;x++)
				{
					T min=img(x,y,z);
					for (auto it=structure_element.begin();it!=structure_element.end();++it)
						if (min>tmp(x+it->x,y+it->y,z+it->z))
							min=tmp(x+it->x,y+it->y,z+it->z);
					img.pixel(x,y,z)=min;
				}
	}
	
} // namespace NRRD
	
#endif // __nrrd_morphology_hxx	
