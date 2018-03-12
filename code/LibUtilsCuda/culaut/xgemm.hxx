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

	/// general matrix multiplication 3x3
	template <typename Matrix3A,typename Matrix3B,typename Matrix3C> __device__ __host__
	inline void xgemm3(const Matrix3A& A, const Matrix3B& B, Matrix3C& C)
	{
		C[0]=A[0]*B[0]+A[3]*B[1]+A[6]*B[2];
		C[1]=A[1]*B[0]+A[4]*B[1]+A[7]*B[2];
		C[2]=A[2]*B[0]+A[5]*B[1]+A[8]*B[2];
		C[3]=A[0]*B[3]+A[3]*B[4]+A[6]*B[5];
		C[4]=A[1]*B[3]+A[4]*B[4]+A[7]*B[5];
		C[5]=A[2]*B[3]+A[5]*B[4]+A[8]*B[5];
		C[6]=A[0]*B[6]+A[3]*B[7]+A[6]*B[8];
		C[7]=A[1]*B[6]+A[4]*B[7]+A[7]*B[8];
		C[8]=A[2]*B[6]+A[5]*B[7]+A[8]*B[8];
	}

	/// general matrix multiplication 4x4
	template <typename Matrix4A,typename Matrix4B,typename Matrix4C> __device__ __host__
	inline void xgemm4(const Matrix4A& A, const Matrix4B& B, Matrix4C& C)
	{
		C[ 0]=A[0]*B[ 0]+A[4]*B[ 1]+A[ 8]*B[ 2]+A[12]*B[ 3];
		C[ 1]=A[1]*B[ 0]+A[5]*B[ 1]+A[ 9]*B[ 2]+A[13]*B[ 3];
		C[ 2]=A[2]*B[ 0]+A[6]*B[ 1]+A[10]*B[ 2]+A[14]*B[ 3];
		C[ 3]=A[3]*B[ 0]+A[7]*B[ 1]+A[11]*B[ 2]+A[15]*B[ 3];
		C[ 4]=A[0]*B[ 4]+A[4]*B[ 5]+A[ 8]*B[ 6]+A[12]*B[ 7];
		C[ 5]=A[1]*B[ 4]+A[5]*B[ 5]+A[ 9]*B[ 6]+A[13]*B[ 7];
		C[ 6]=A[2]*B[ 4]+A[6]*B[ 5]+A[10]*B[ 6]+A[14]*B[ 7];
		C[ 7]=A[3]*B[ 4]+A[7]*B[ 5]+A[11]*B[ 6]+A[15]*B[ 7];
		C[ 8]=A[0]*B[ 8]+A[4]*B[ 9]+A[ 8]*B[10]+A[12]*B[11];
		C[ 9]=A[1]*B[ 8]+A[5]*B[ 9]+A[ 9]*B[10]+A[13]*B[11];
		C[10]=A[2]*B[ 8]+A[6]*B[ 9]+A[10]*B[10]+A[14]*B[11];
		C[11]=A[3]*B[ 8]+A[7]*B[ 9]+A[11]*B[10]+A[15]*B[11];
		C[12]=A[0]*B[12]+A[4]*B[13]+A[ 8]*B[14]+A[12]*B[15];
		C[13]=A[1]*B[12]+A[5]*B[13]+A[ 9]*B[14]+A[13]*B[15];
		C[14]=A[2]*B[12]+A[6]*B[13]+A[10]*B[14]+A[14]*B[15];
		C[15]=A[3]*B[12]+A[7]*B[13]+A[11]*B[14]+A[15]*B[15];
	}

	/// General matrix multiplication 4x4 transposed times 4x4
	template <typename Matrix4A,typename Matrix4B,typename Matrix4C> __device__ __host__
	inline void xgemm4t4(const Matrix4A& A, const Matrix4B& B, Matrix4C& C)
	{
		C[ 0]=A[ 0]*B[ 0] + A[ 1]*B[ 1] + A[ 2]*B[ 2] + A[ 3]*B[ 3];
		C[ 1]=A[ 4]*B[ 0] + A[ 5]*B[ 1] + A[ 6]*B[ 2] + A[ 7]*B[ 3];
		C[ 2]=A[ 8]*B[ 0] + A[ 9]*B[ 1] + A[10]*B[ 2] + A[11]*B[ 3];
		C[ 3]=A[12]*B[ 0] + A[13]*B[ 1] + A[14]*B[ 2] + A[15]*B[ 3];
		C[ 4]=A[ 0]*B[ 4] + A[ 1]*B[ 5] + A[ 2]*B[ 6] + A[ 3]*B[ 7];
		C[ 5]=A[ 4]*B[ 4] + A[ 5]*B[ 5] + A[ 6]*B[ 6] + A[ 7]*B[ 7];
		C[ 6]=A[ 8]*B[ 4] + A[ 9]*B[ 5] + A[10]*B[ 6] + A[11]*B[ 7];
		C[ 7]=A[12]*B[ 4] + A[13]*B[ 5] + A[14]*B[ 6] + A[15]*B[ 7];
		C[ 8]=A[ 0]*B[ 8] + A[ 1]*B[ 9] + A[ 2]*B[10] + A[ 3]*B[11];
		C[ 9]=A[ 4]*B[ 8] + A[ 5]*B[ 9] + A[ 6]*B[10] + A[ 7]*B[11];
		C[10]=A[ 8]*B[ 8] + A[ 9]*B[ 9] + A[10]*B[10] + A[11]*B[11];
		C[11]=A[12]*B[ 8] + A[13]*B[ 9] + A[14]*B[10] + A[15]*B[11];
		C[12]=A[ 0]*B[12] + A[ 1]*B[13] + A[ 2]*B[14] + A[ 3]*B[15];
		C[13]=A[ 4]*B[12] + A[ 5]*B[13] + A[ 6]*B[14] + A[ 7]*B[15];
		C[14]=A[ 8]*B[12] + A[ 9]*B[13] + A[10]*B[14] + A[11]*B[15];
		C[15]=A[12]*B[12] + A[13]*B[13] + A[14]*B[14] + A[15]*B[15];
	}

	/// General Matrix Multiplication 3x3 * 3x4=3x4
	template <typename Matrix33,typename Matrix34B,typename Matrix34C> __device__ __host__
	inline void xgemml34(const Matrix33& H, const Matrix34B& P, Matrix34C& HP)
	{
		HP[ 0]=P[ 0]*H[ 0]+P[ 1]*H[3]+P[ 2]*H[ 6];
		HP[ 1]=P[ 0]*H[ 1]+P[ 1]*H[4]+P[ 2]*H[ 7];
		HP[ 2]=P[ 0]*H[ 2]+P[ 1]*H[5]+P[ 2]*H[ 9];
		HP[ 3]=P[ 3]*H[ 0]+P[ 4]*H[3]+P[ 5]*H[ 6];
		HP[ 4]=P[ 3]*H[ 1]+P[ 4]*H[4]+P[ 5]*H[ 7];
		HP[ 5]=P[ 3]*H[ 2]+P[ 4]*H[5]+P[ 5]*H[ 9];
		HP[ 6]=P[ 6]*H[ 0]+P[ 7]*H[3]+P[ 8]*H[ 6];
		HP[ 7]=P[ 6]*H[ 1]+P[ 7]*H[4]+P[ 8]*H[ 7];
		HP[ 8]=P[ 6]*H[ 2]+P[ 7]*H[5]+P[ 8]*H[ 9];
		HP[ 9]=P[ 9]*H[ 0]+P[10]*H[3]+P[11]*H[ 6];
		HP[10]=P[ 9]*H[ 1]+P[10]*H[4]+P[11]*H[ 7];
		HP[11]=P[ 9]*H[ 2]+P[10]*H[5]+P[11]*H[ 9];
	}

	/// General Matrix Multiplication 3x4 * 4x4=3x4
	template <typename Matrix34A,typename Matrix44,typename Matrix34C> __device__ __host__
	inline void xgemmr34(const Matrix34A& P, const Matrix44& T, Matrix34C& PT)
	{
		PT[ 0]=P[0]*T[ 0]+P[3]*T[ 1]+P[6]*T[ 2]+P[ 9]*T[ 3];
		PT[ 1]=P[1]*T[ 0]+P[4]*T[ 1]+P[7]*T[ 2]+P[10]*T[ 3];
		PT[ 2]=P[2]*T[ 0]+P[5]*T[ 1]+P[8]*T[ 2]+P[11]*T[ 3];
		PT[ 3]=P[0]*T[ 4]+P[3]*T[ 5]+P[6]*T[ 6]+P[ 9]*T[ 7];
		PT[ 4]=P[1]*T[ 4]+P[4]*T[ 5]+P[7]*T[ 6]+P[10]*T[ 7];
		PT[ 5]=P[2]*T[ 4]+P[5]*T[ 5]+P[8]*T[ 6]+P[11]*T[ 7];
		PT[ 6]=P[0]*T[ 8]+P[3]*T[ 9]+P[6]*T[10]+P[ 9]*T[11];
		PT[ 7]=P[1]*T[ 8]+P[4]*T[ 9]+P[7]*T[10]+P[10]*T[11];
		PT[ 8]=P[2]*T[ 8]+P[5]*T[ 9]+P[8]*T[10]+P[11]*T[11];
		PT[ 9]=P[0]*T[12]+P[3]*T[13]+P[6]*T[14]+P[ 9]*T[15];
		PT[10]=P[1]*T[12]+P[4]*T[13]+P[7]*T[14]+P[10]*T[15];
		PT[11]=P[2]*T[12]+P[5]*T[13]+P[8]*T[14]+P[11]*T[15];
	}
	
	/// Right multiplication 3x4 matrix with transpose of 4x4 matrix 3x4 * (4x4)^T =3x4 and left multiplication with 3x3 matrix  3x3 * 3x4 =3x4 
	template <typename Matrix33,typename Matrix34A,typename Matrix44,typename Matrix34B> __device__ __host__
	inline void xgemmlr34t(const Matrix33& H, const Matrix34A& P, const Matrix44& T, Matrix34B HPTt)
	{
		HPTt[ 0]=T[0]*(H[0]*P[0]+H[3]*P[1]+H[6]*P[2])+T[4]*(H[0]*P[3]+H[3]*P[4]+H[6]*P[5])+T[ 8]*(H[0]*P[6]+H[3]*P[7]+H[6]*P[8])+T[12]*(H[0]*P[9]+H[3]*P[10]+H[6]*P[11]);
		HPTt[ 1]=T[0]*(H[1]*P[0]+H[4]*P[1]+H[7]*P[2])+T[4]*(H[1]*P[3]+H[4]*P[4]+H[7]*P[5])+T[ 8]*(H[1]*P[6]+H[4]*P[7]+H[7]*P[8])+T[12]*(H[1]*P[9]+H[4]*P[10]+H[7]*P[11]);
		HPTt[ 2]=T[0]*(H[2]*P[0]+H[5]*P[1]+H[8]*P[2])+T[4]*(H[2]*P[3]+H[5]*P[4]+H[8]*P[5])+T[ 8]*(H[2]*P[6]+H[5]*P[7]+H[8]*P[8])+T[12]*(H[2]*P[9]+H[5]*P[10]+H[8]*P[11]);
		HPTt[ 3]=T[1]*(H[0]*P[0]+H[3]*P[1]+H[6]*P[2])+T[5]*(H[0]*P[3]+H[3]*P[4]+H[6]*P[5])+T[ 9]*(H[0]*P[6]+H[3]*P[7]+H[6]*P[8])+T[13]*(H[0]*P[9]+H[3]*P[10]+H[6]*P[11]);
		HPTt[ 4]=T[1]*(H[1]*P[0]+H[4]*P[1]+H[7]*P[2])+T[5]*(H[1]*P[3]+H[4]*P[4]+H[7]*P[5])+T[ 9]*(H[1]*P[6]+H[4]*P[7]+H[7]*P[8])+T[13]*(H[1]*P[9]+H[4]*P[10]+H[7]*P[11]);
		HPTt[ 5]=T[1]*(H[2]*P[0]+H[5]*P[1]+H[8]*P[2])+T[5]*(H[2]*P[3]+H[5]*P[4]+H[8]*P[5])+T[ 9]*(H[2]*P[6]+H[5]*P[7]+H[8]*P[8])+T[13]*(H[2]*P[9]+H[5]*P[10]+H[8]*P[11]);
		HPTt[ 6]=T[2]*(H[0]*P[0]+H[3]*P[1]+H[6]*P[2])+T[6]*(H[0]*P[3]+H[3]*P[4]+H[6]*P[5])+T[10]*(H[0]*P[6]+H[3]*P[7]+H[6]*P[8])+T[14]*(H[0]*P[9]+H[3]*P[10]+H[6]*P[11]);
		HPTt[ 7]=T[2]*(H[1]*P[0]+H[4]*P[1]+H[7]*P[2])+T[6]*(H[1]*P[3]+H[4]*P[4]+H[7]*P[5])+T[10]*(H[1]*P[6]+H[4]*P[7]+H[7]*P[8])+T[14]*(H[1]*P[9]+H[4]*P[10]+H[7]*P[11]);
		HPTt[ 8]=T[2]*(H[2]*P[0]+H[5]*P[1]+H[8]*P[2])+T[6]*(H[2]*P[3]+H[5]*P[4]+H[8]*P[5])+T[10]*(H[2]*P[6]+H[5]*P[7]+H[8]*P[8])+T[14]*(H[2]*P[9]+H[5]*P[10]+H[8]*P[11]);
		HPTt[ 9]=T[3]*(H[0]*P[0]+H[3]*P[1]+H[6]*P[2])+T[7]*(H[0]*P[3]+H[3]*P[4]+H[6]*P[5])+T[11]*(H[0]*P[6]+H[3]*P[7]+H[6]*P[8])+T[15]*(H[0]*P[9]+H[3]*P[10]+H[6]*P[11]);
		HPTt[10]=T[3]*(H[1]*P[0]+H[4]*P[1]+H[7]*P[2])+T[7]*(H[1]*P[3]+H[4]*P[4]+H[7]*P[5])+T[11]*(H[1]*P[6]+H[4]*P[7]+H[7]*P[8])+T[15]*(H[1]*P[9]+H[4]*P[10]+H[7]*P[11]);
		HPTt[11]=T[3]*(H[2]*P[0]+H[5]*P[1]+H[8]*P[2])+T[7]*(H[2]*P[3]+H[5]*P[4]+H[8]*P[5])+T[11]*(H[2]*P[6]+H[5]*P[7]+H[8]*P[8])+T[15]*(H[2]*P[9]+H[5]*P[10]+H[8]*P[11]);
	}


	/// Multiplication of 3x4 matrix with a 4x3 matrix 3x4 * 4x3 = 3x3
	template <typename Matrix34,typename Matrix43,typename Matrix33>  __device__ __host__
	inline void xgemm3443(const Matrix34& P, const Matrix43& K, Matrix33& O)
	{
		O[0]=K[0]*P[0]+K[1]*P[3]+K[ 2]*P[6]+K[ 3]*P[ 9];
		O[1]=K[0]*P[1]+K[1]*P[4]+K[ 2]*P[7]+K[ 3]*P[10];
		O[2]=K[0]*P[2]+K[1]*P[5]+K[ 2]*P[8]+K[ 3]*P[11];
		O[3]=K[4]*P[0]+K[5]*P[3]+K[ 6]*P[6]+K[ 7]*P[ 9];
		O[4]=K[4]*P[1]+K[5]*P[4]+K[ 6]*P[7]+K[ 7]*P[10];
		O[5]=K[4]*P[2]+K[5]*P[5]+K[ 6]*P[8]+K[ 7]*P[11];
		O[6]=K[8]*P[0]+K[9]*P[3]+K[10]*P[6]+K[11]*P[ 9];
		O[7]=K[8]*P[1]+K[9]*P[4]+K[10]*P[7]+K[11]*P[10];
		O[8]=K[8]*P[2]+K[9]*P[5]+K[10]*P[8]+K[11]*P[11];
	}

	/// Multiplication of 3x4 matrix with a 4x2 matrix 3x4 * 4x2 = 3x2
	template <typename Matrix34,typename Matrix42,typename Matrix32>  __device__ __host__
	inline void xgemm3442(const Matrix34& P, const Matrix42& K, Matrix32& O)
	{
		O[0]=K[0]*P[0] + K[1]*P[3] + K[2]*P[6] + K[3]*P[ 9];
		O[1]=K[0]*P[1] + K[1]*P[4] + K[2]*P[7] + K[3]*P[10];
		O[2]=K[0]*P[2] + K[1]*P[5] + K[2]*P[8] + K[3]*P[11];
		O[3]=K[4]*P[0] + K[5]*P[3] + K[6]*P[6] + K[7]*P[ 9];
		O[4]=K[4]*P[1] + K[5]*P[4] + K[6]*P[7] + K[7]*P[10];
		O[5]=K[4]*P[2] + K[5]*P[5] + K[6]*P[8] + K[7]*P[11];
	}

} // namespace culaut

#ifdef udef__device__
#undef __device__
#endif
#ifdef udef__host__
#undef __host__
#endif

#endif // __xgemm_hxx
