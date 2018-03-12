#ifndef __x_projective_geometry_hxx
#define __x_projective_geometry_hxx

#ifndef __device__
#define __device__
#define udef__device__
#endif
#ifndef __host__
#define __host__
#define udef__host__
#endif

namespace culaut
{

	/// Join two points to form a line. L= A join B
	template <typename T> __device__ __host__ inline
	void join_pluecker(T A[4], T B[4], T L[6])
	{
		L[0]=A[0]*B[1]-A[1]*B[0];
		L[1]=A[0]*B[2]-A[2]*B[0];
		L[2]=A[0]*B[3]-A[3]*B[0];
		L[3]=A[1]*B[2]-A[2]*B[1];
		L[4]=A[1]*B[3]-A[3]*B[1];
		L[5]=A[2]*B[3]-A[3]*B[2];
	}

	/// Meet a line and a plane to form a point. X= L meet P
	template <typename T> __device__ __host__ inline
	void meet_pluecker(T L[6], T P[4], T X[4])
	{
		X[0]=	            - P[1]*L[0] - P[2]*L[1] - P[3]*L[2] ;
		X[1]=	+ P[0]*L[0]             - P[2]*L[3] - P[3]*L[4] ;
		X[2]=	+ P[0]*L[1] + P[1]*L[3]             - P[3]*L[5] ;
		X[3]=	+ P[0]*L[2] + P[1]*L[4] + P[2]*L[5]				;
	}

	template <typename T> __device__ __host__ inline
	void pluecker_closest_point_to_origin(T L[6], T O[4])
	{
		O[0]=L[4]*L[0]+L[1]*L[5];
		O[1]=-L[0]*L[2]+L[3]*L[5];
		O[2]=-L[1]*L[2]-L[3]*L[4];
		O[3]=-L[2]*L[2]-L[4]*L[4]-L[5]*L[5];
	}

} // namespace culaut

#endif // __x_projective_geometry_hxx
