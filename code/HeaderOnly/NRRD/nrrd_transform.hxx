// Created by A. Aichert
#ifndef __nrrd_transform_hxx
#define __nrrd_transform_hxx

#include "nrrd_image.hxx"
// 2do replace LAUT with GLM or similar
#include "../deprecated/laut.hxx"

#define HAVE_OPENMP

#ifdef HAVE_OPENMP
#include <omp.h>
#endif

namespace NRRD {

/// Transform a volume given a projective transformation.
/// Coordinates in millimeters with the lower back left corner as origin.
/// Spacing provided is(x/y/z) and os(x/y/z), number of voxels as provided.
template <typename Type>
void transform_volume(
	const NRRD::Image<Type>& I, NRRD::Image<Type>& O, double* _T,
	double isx=1, double isy=1, double isz=1,
	double osx=1, double osy=1, double osz=1)
{
	LAUT::Ref<double> T(_T,4,4);
	// Invert for backward warping
	LAUT::Mat<double,4,4> Tinv;
	LAUT::xgeinv4(T,Tinv);

	// Image sizes, for readibility. 
	int onx=O.size(0),ony=O.size(1),onz=O.size(2);

	#ifdef HAVE_OPENMP
	#pragma omp parallel for
	#endif
	for (int z=0;z<onz;z++)
	{
		for (int y=0;y<ony;y++)
			for (int x=0;x<onx;x++)
			{
				// Projective 3-vector for point in output volume in [mm]
				double ovecmm[]={osx*x,osy*y,osz*z,1};
				// Project to inpuit volume [mm]
				double ivecmm[4];
				LAUT::xgemv4(Tinv,ovecmm,ivecmm);
				// De-homogenize and scale to voxels
				double ivec[]={
					ivecmm[0]/(isx*ivecmm[3]),
					ivecmm[1]/(isz*ivecmm[3]),
					ivecmm[2]/(isz*ivecmm[3])
				};
				double sample=I(ivec[0],ivec[1],ivec[2]);
				O.pixel(x,y,z)=(Type)sample;
			}
	}
}

} // namespace NRRD

#endif // __nrrd_transform_hxx
