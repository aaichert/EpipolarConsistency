#ifndef __epipolar_consistency_common
#define __epipolar_consistency_common
// Created by A. Aichert on thu Jan 25th 2018
// Common functions for epipolar consistency.
// Contains:
// get_ij: iterating over pairs.
//    Compute integers i and j from an ij<n*(n-1)/2, such that every combination i<j<n is obtained once
// shiftOriginAndNormlaize: Translating line coordinates.
//    Take two lines given by 3 values with a stride of 4, translate by (x,y) and scale bothm such that the first is in HNF.
// computeK01: Parametrize epipolar plane bundle.
//    Compute 3x2 matrices mapping [cos(kappa) sin(kappa)] to corresponding epipolar lines l0_kappa and l1_kappa directly
//    Also computes distance of the stereo baseline to the origin, appropriate sampling of kappa and range of kappa.
// lineToSampleDtr: Compute location in Radon intermedaite function correspsonding to a line
//    Angle is returned in line[0] in a range from -1 to 1 (corresponding to -Pi to Pi)
//    Distance is returned in line[1] in range 0 to 1 (corresponding to zero and image diagonal pixels)

#ifndef __device__
#define __device__
#define udef__device__
#endif
#ifndef __host__
#define __host__
#define udef__host__
#endif

#include <cmath>


/// f(x)=1-x^2+x^4 is zero at +/-1, has zero derivative at +/-1 and a maxiumum at f(0)=1; Values outside [-1,1] are clamped to zero. 
template <typename T> __device__ __host__ inline T weighting(T x)
{
	if (x<T(-1)||x>T(1)) return T(0);
	T xx=x*x;
	return T(1)-2*xx+xx*xx;
}

/// General matrix multiplication MxO times OxN result is MxN
template <typename T, int M, int O, int N> __device__ __host__ inline
void xgemm(const T* A, const T* B, T* C)
{
	for (int i=0;i<M;i++)
		for (int j=0;j<N;j++)
		{
			T sum=0;
			for (int s=0;s<O;s++)
				sum+=A[s*M+i]*B[j*O+s];
			C[j*M+i]=sum;
		}
}

// maps ij to pairs i,j with i,j<n and i<j
__device__ __host__ inline void get_ij(int ij, short n, short&i, short&j)
{
	//function [i j]= ij(k,n)
	//	i=0;
	//	k=k+1;
	//	while (k>n-i-1)
	//		i=i+1;
	//		k=k-(n-i);
	//	end % while
	//	j=k+i;
	//end % function
	//function A=ijmat(n)
	//	A=zeros(n);
	//	N=n*(n-1)/2;
	//	for k=0:N-1
	//		[i j]=ij(k,n);
	//		A(i+1,j+1)=k;
	//	end % for
	//end % function
	i=0;
	int k=ij+1;
	while (k>n-i-1)
	{
		i++;
		k-=(n-i);
	}
	j=k+i;
}

/// Applies a homograohy of the image and scales the normal of l^0 (for kappa=0)
__device__ __host__ inline void shiftOriginAndNormlaize(float x, float y, float* Ki)
{
	// Multiplication with inv(H)', where H is a translation by (-x,-y)
	Ki[2]+=x*Ki[0]+y*Ki[1];
	Ki[5]+=x*Ki[3]+y*Ki[4];
	float s0=sqrtf(Ki[0]*Ki[0]+Ki[1]*Ki[1]); // length of normal of l0 (l0 is always finite)
	for (int i=0;i<6;i++) Ki[i]/=s0;
}

// Compute 3x2 matrices mapping [cos(kappa) sin(kappa)] to l0_kappa and l1_kappa directly
// Will store basline distance in K0[6], while dkappa and kappa_max will be stored in K1[6] and K1[7] respectively.
__device__ __host__ inline void computeK01(
	float	n_x2,				//< Half image width. Lines shall be relative to center pixel.
	float	n_y2,				//< Half image height. Lines shall be relative to center pixel.
	float*	const  C0,			//< Source positions 0
	float*	const  C1,			//< Source positions 1
	float*	P0invT,				//< Projection 0 pseudo inverse transpose
	float*	P1invT,				//< Projection 1 pseudo inverse transpose
	float	object_radius_mm,	//< Approximate size of object in mm (determines angular range)
	float	num_samples,		//< Number of samples to be drawn (in the order of image diagonal pixels)
	float	dkappa,				//< Optional: fixed dkappa. default is to pass zero.
	float*	K0,					//< out: P0invT*BxdA
	float*	K1					//< out: P1invT*BxdA
	)
{
	// Ignore equal projection matrices
	if (C0==C1)
	{
		for (int i=0;i<8;i++) K0[i]=0;
		for (int i=0;i<8;i++) K1[i]=0;
		return;
	}
	// Compute Plücker coordinates of the baseline
	float B01=C0[0]*C1[1]-C0[1]*C1[0];
	float B02=C0[0]*C1[2]-C0[2]*C1[0];
	float B03=C0[0]*C1[3]-C0[3]*C1[0];
	float B12=C0[1]*C1[2]-C0[2]*C1[1];
	float B13=C0[1]*C1[3]-C0[3]*C1[1];
	float B23=C0[2]*C1[3]-C0[3]*C1[2];
	// Normalize by line moment of baseline
	const float s2=sqrtf(B12*B12+B02*B02+B01*B01);
	const float s3=sqrtf(B03*B03+B13*B13+B23*B23); //< Distance between source positions if C0[3]==C1[3]
	// K is a 4x2 matrix mapping [cos(kappa) sin(kappa)] to epipolar plane E_kappa.
	// It consists of two planes E0 and E90, (i.e. for kappa=0 and kappa=90°) both of which contain the baseline.
	float K[] = {
	    + B12/s2                    , - B02/s2                    , + B01/s2                    , 0,	  //< E0  Plane through origin
	   (- B01*B13 - B02*B23)/(s2*s3),(+ B01*B03 - B12*B23)/(s2*s3),(+ B02*B03 + B12*B13)/(s2*s3), -s2/s3, //< E90 Plane with maximal distance to origin
	};
	// Multiply by pseudoinverse transposed to map epipolar plane E_kappa to lines l_kappa
	xgemm<float,3,4,2>(P0invT,K,K0); // K0 is 3x2 and maps [cos(kappa) sin(kappa)] to l0_kappa
	xgemm<float,3,4,2>(P1invT,K,K1); // K1 is 3x2 and maps [cos(kappa) sin(kappa)] to l1_kappa
	// We want the lines to be relative to the image center, instead of the image corner.
	shiftOriginAndNormlaize(n_x2,n_y2,K0);
	shiftOriginAndNormlaize(n_x2,n_y2,K1);
	// Distance of baseline to reference point
	K0[6]=s2/s3;
	// Angle between source positions and reference point
	K0[7]=-2.0f*atan2f(-0.5f*s3,s2/s3);
	// If the baseline intersects the object, we return a half circle.
	const float Pi=3.14159265359f;
	if (K0[6]<=object_radius_mm)
		K1[7]=0.5f*Pi;
	else// Define range of kappa based on baline distance and radius of object
		K1[7]=asinf(object_radius_mm/K0[6]);
	// Finally, compute dkappa for appropriate sampling, unless user has fixed its value.
	if (dkappa<=0.f) K1[6]=2.f*K1[7]/num_samples;
	else             K1[6]=dkappa;
}

/// Compute location to be sampled within Radon intermediate transform. line[0] -> angle, line[1] distance. Returns true if periodicity had to be used to get the sample in 0-1 range.
template <typename Line>
__device__ __host__ inline bool lineToSampleDtr(Line& line, float range_t)
{
	const float Pi=3.14159265359f;
	// Length or normal vector
	float length=sqrtf(line[0]*line[0]+line[1]*line[1]);
	// Angle between line normal and x-axis (scaled from 0 to +2)
	line[0]=atan2f(line[1],line[0])/Pi;
	if (line[0]<0) line[0]+=2;
	// Distance to the origin (scaled to DTR bins)
	line[1]=-(line[2]/length)/range_t+0.5f;
	// Account for periodicity	
	if (line[0]>1)
	{
		line[0]=line[0]-1.f;
		line[1]=1.f-line[1];
		return true;
	}
	return false;
}

#ifdef udef__device__
#undef __device__
#endif
#ifdef udef__host__
#undef __host__
#endif

#endif // __epipolar_consistency_common
