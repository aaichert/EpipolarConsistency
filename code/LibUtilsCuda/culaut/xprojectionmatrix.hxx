#ifndef __xprojectionmatrix_hxx
#define __xprojectionmatrix_hxx

#include "xgeinv.hxx"

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
		
	/// Pseudoinverse of 3x4 matrix for backprojection of image points. Transpose is returned.
	template <typename Scalar,typename Scalar2> __device__ __host__
	inline void projection_matrix_pseudoinverse_transpose(const Scalar *P, Scalar2 *PinvT)
	{
		Scalar PPTinv[9];
		{
			// Compute P*P'
			Scalar PPT[]={
				P[0]*P[0] + P[3]*P[3] + P[6]*P[6] + P[9 ]*P[9 ],
				P[0]*P[1] + P[3]*P[4] + P[6]*P[7] + P[9 ]*P[10],
				P[0]*P[2] + P[3]*P[5] + P[6]*P[8] + P[9 ]*P[11],
				1,
				P[1]*P[1] + P[4]*P[4] + P[7]*P[7] + P[10]*P[10],
				P[1]*P[2] + P[4]*P[5] + P[7]*P[8] + P[10]*P[11],
				2,
				5,
				P[2]*P[2] + P[5]*P[5] + P[8]*P[8] + P[11]*P[11]
			};
			PPT[3]=PPT[1];PPT[6]=PPT[2];PPT[7]=PPT[5];
			// Compute inverse of symmetric matrix
			xgeinv<Scalar,3>(PPT,PPTinv);
		}
		PinvT[0 ] = (Scalar2)(P[0]*PPTinv[0] + P[1 ]*PPTinv[1] + P[2 ]*PPTinv[2]);
		PinvT[1 ] = (Scalar2)(P[0]*PPTinv[3] + P[1 ]*PPTinv[4] + P[2 ]*PPTinv[5]);
		PinvT[2 ] = (Scalar2)(P[0]*PPTinv[6] + P[1 ]*PPTinv[7] + P[2 ]*PPTinv[8]);
		PinvT[3 ] = (Scalar2)(P[3]*PPTinv[0] + P[4 ]*PPTinv[1] + P[5 ]*PPTinv[2]);
		PinvT[4 ] = (Scalar2)(P[3]*PPTinv[3] + P[4 ]*PPTinv[4] + P[5 ]*PPTinv[5]);
		PinvT[5 ] = (Scalar2)(P[3]*PPTinv[6] + P[4 ]*PPTinv[7] + P[5 ]*PPTinv[8]);
		PinvT[6 ] = (Scalar2)(P[6]*PPTinv[0] + P[7 ]*PPTinv[1] + P[8 ]*PPTinv[2]);
		PinvT[7 ] = (Scalar2)(P[6]*PPTinv[3] + P[7 ]*PPTinv[4] + P[8 ]*PPTinv[5]);
		PinvT[8 ] = (Scalar2)(P[6]*PPTinv[6] + P[7 ]*PPTinv[7] + P[8 ]*PPTinv[8]);
		PinvT[9 ] = (Scalar2)(P[9]*PPTinv[0] + P[10]*PPTinv[1] + P[11]*PPTinv[2]);
		PinvT[10] = (Scalar2)(P[9]*PPTinv[3] + P[10]*PPTinv[4] + P[11]*PPTinv[5]);
		PinvT[11] = (Scalar2)(P[9]*PPTinv[6] + P[10]*PPTinv[7] + P[11]*PPTinv[8]);
	}

	/// Pseudoinverse of 3x4 matrix for backprojection of image points. Result overwrites input
	template <typename Scalar> __device__ __host__
	inline void projection_matrix_pseudoinverse_transpose_inplace(Scalar *P)
	{
		Scalar PPTinv[9];
		// Compute P*P'
		Scalar PPT[]={
			P[0]*P[0] + P[3]*P[3] + P[6]*P[6] + P[9 ]*P[9 ],
			P[0]*P[1] + P[3]*P[4] + P[6]*P[7] + P[9 ]*P[10],
			P[0]*P[2] + P[3]*P[5] + P[6]*P[8] + P[9 ]*P[11],
			1,
			P[1]*P[1] + P[4]*P[4] + P[7]*P[7] + P[10]*P[10],
			P[1]*P[2] + P[4]*P[5] + P[7]*P[8] + P[10]*P[11],
			2,
			5,
			P[2]*P[2] + P[5]*P[5] + P[8]*P[8] + P[11]*P[11],
			0,0,0 // used for matrix multiplication
		};
		PPT[3]=PPT[1];PPT[6]=PPT[2];PPT[7]=PPT[5];
		// Compute inverse of symmetric matrix
		xgeinv<Scalar,3>(PPT,PPTinv);
		// Multiply with transpose of P
		PPT[0 ] = (P[0]*PPTinv[0] + P[1 ]*PPTinv[1] + P[2 ]*PPTinv[2]);
		PPT[1 ] = (P[0]*PPTinv[3] + P[1 ]*PPTinv[4] + P[2 ]*PPTinv[5]);
		PPT[2 ] = (P[0]*PPTinv[6] + P[1 ]*PPTinv[7] + P[2 ]*PPTinv[8]);
		PPT[3 ] = (P[3]*PPTinv[0] + P[4 ]*PPTinv[1] + P[5 ]*PPTinv[2]);
		PPT[4 ] = (P[3]*PPTinv[3] + P[4 ]*PPTinv[4] + P[5 ]*PPTinv[5]);
		PPT[5 ] = (P[3]*PPTinv[6] + P[4 ]*PPTinv[7] + P[5 ]*PPTinv[8]);
		PPT[6 ] = (P[6]*PPTinv[0] + P[7 ]*PPTinv[1] + P[8 ]*PPTinv[2]);
		PPT[7 ] = (P[6]*PPTinv[3] + P[7 ]*PPTinv[4] + P[8 ]*PPTinv[5]);
		PPT[8 ] = (P[6]*PPTinv[6] + P[7 ]*PPTinv[7] + P[8 ]*PPTinv[8]);
		PPT[9 ] = (P[9]*PPTinv[0] + P[10]*PPTinv[1] + P[11]*PPTinv[2]);
		PPT[10] = (P[9]*PPTinv[3] + P[10]*PPTinv[4] + P[11]*PPTinv[5]);
		PPT[11] = (P[9]*PPTinv[6] + P[10]*PPTinv[7] + P[11]*PPTinv[8]);
		culaut::xvcpy<Scalar,Scalar,12>(P,PPT);
	}

	/// Computation of the center of projection from 3x4 projection matrix
	template <typename Scalar,typename Scalar2> __device__ __host__
	inline void projection_matrix_source_position(const Scalar *P, Scalar2 *C)
	{
		// Transpose of A into left part of R
		Scalar Q[4*4], R[4*4];
		for (int i=0;i<3;i++)
			for (int j=0;j<4;j++)
				R[j+4*i]=P[i+3*j];
		for (int j=0;j<4;j++)
			R[j+4*3]=0;
		xsqqr<Scalar,4>(R,Q);
		for (int i=0;i<4;i++)
			C[i]=(Scalar2)(Q[i+4*3]/Q[3+4*3]);		
	}

} // namespace culaut

#ifdef udef__device__
#undef __device__
#endif
#ifdef udef__host__
#undef __host__
#endif

#endif // __xprojectionmatrix_hxx
