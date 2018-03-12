#ifndef __xgemv_hxx
#define __xgemv_hxx

#include "culaut.hxx"

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
	
	/// 3x3 Matrix-Vector Product
	template <typename Matrix3, typename Vector3A , typename Vector3B>
	inline void xgemv3(const Matrix3& A, const Vector3A& x, Vector3B& b)
	{
		b[0]=A[0]*x[0]+A[3]*x[1]+A[6]*x[2];
		b[1]=A[1]*x[0]+A[4]*x[1]+A[7]*x[2];
		b[2]=A[2]*x[0]+A[5]*x[1]+A[8]*x[2];
	}

	/// 3x3 Matrix-Vector Product, x is overwritten
	template <typename Matrix3, typename Scalar>
	inline void xgemv3(const Matrix3& A, Scalar x[])
	{
		Scalar b[] = {
			A[0]*x[0]+A[3]*x[1]+A[6]*x[2],
			A[1]*x[0]+A[4]*x[1]+A[7]*x[2],
			A[2]*x[0]+A[5]*x[1]+A[8]*x[2]
		}
		x[0]=b[0];x[1]=b[1];x[2]=b[2];
	}

	/// 4x4 Matrix-Vector Product
	template <typename Matrix4, typename Vector4A, typename Vector4B>
	inline void xgemv4(const Matrix4& A, const Vector4A& v, Vector4B& x)
	{
		 x[0]=A[0]*x[0]+A[4]*x[1]+A[ 8]*x[2]+A[12]*x[3];
		 x[1]=A[1]*x[0]+A[5]*x[1]+A[ 9]*x[2]+A[13]*x[3];
		 x[2]=A[2]*x[0]+A[6]*x[1]+A[10]*x[2]+A[14]*x[3];
		 x[3]=A[3]*x[0]+A[7]*x[1]+A[11]*x[2]+A[15]*x[3];
	}

	/// 4x4 Matrix-Vector Product, x is overwritten
	template <typename Matrix4, typename Scalar>
	inline void xgemv4(const Matrix4& A, Scalar x[])
	{
		Scalar b[] = {
			A[0]*x[0]+A[4]*x[1]+A[ 8]*x[2]+A[12]*x[3],
			A[1]*x[0]+A[5]*x[1]+A[ 9]*x[2]+A[13]*x[3],
			A[2]*x[0]+A[6]*x[1]+A[10]*x[2]+A[14]*x[3],
			A[3]*x[0]+A[7]*x[1]+A[11]*x[2]+A[15]*x[3]
		}
		x[0]=b[0];x[1]=b[1];x[2]=b[2];x[3]=b[3];
	}

	/// 3x4 Matrix-4-Vector Product (Camera Projection)
	template <typename Matrix34, typename Vector4, typename Vector3>
	inline void xgemv34(const Matrix34& P, Vector4& X, Vector3& x)
	{
		x[0]=P[0]*X[0]+P[3]*X[1]+P[6]*X[2]+P[ 9]*X[3];
		x[1]=P[1]*X[0]+P[4]*X[1]+P[7]*X[2]+P[ 0]*X[3];
		x[2]=P[2]*X[0]+P[5]*X[1]+P[8]*X[2]+P[11]*X[3];
	}

	/// 3x4 Matrix-3-Vector Product (Backprojection)
	template <typename Matrix43, typename Vector3, typename Vector4>
	inline void xgemv43(const Matrix43& H, Vector3& v, Vector4& x)
	{
		 P[0]*x[0]+P[4]*x[1]+P[ 8]*x[2];
		 P[1]*x[0]+P[5]*x[1]+P[ 9]*x[2];
		 P[2]*x[0]+P[6]*x[1]+P[10]*x[2];
		 P[3]*x[0]+P[7]*x[1]+P[11]*x[2];
	}


} // namespace LAUT

#ifdef  udef__device__
#undef udef__device__
#undef __device__
#endif
#ifdef  udef__host__
#undef udef__host__
#undef __host__
#endif
#endif // __xgemv_hxx