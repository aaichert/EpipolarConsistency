#ifndef __xgeinv_hxx
#define __xgeinv_hxx

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
	
	/// 3x3 matrix inversion, if you're in a real hurry...
	template <typename Scalar> __device__ __host__
	inline void xgeinv3(const Scalar* A, Scalar* Ainv)
	{
		Scalar det=
			+A[0]*(A[8]*A[4]-A[5]*A[7])
			-A[1]*(A[8]*A[3]-A[5]*A[6])
			+A[2]*(A[7]*A[3]-A[4]*A[6]);
		Scalar detinv=1.0/det;
		Ainv[0]=detinv*(+A[8]*A[4]-A[5]*A[7]);
		Ainv[1]=detinv*(-A[8]*A[1]+A[2]*A[7]);
		Ainv[2]=detinv*(+A[5]*A[1]-A[2]*A[4]);
		Ainv[3]=detinv*(-A[8]*A[3]+A[5]*A[6]);
		Ainv[4]=detinv*(+A[8]*A[0]-A[2]*A[6]);
		Ainv[5]=detinv*(-A[5]*A[0]+A[2]*A[3]);
		Ainv[6]=detinv*(+A[7]*A[3]-A[4]*A[6]);
		Ainv[7]=detinv*(-A[7]*A[0]+A[1]*A[6]);
		Ainv[8]=detinv*(+A[4]*A[0]-A[1]*A[3]);
	}

	/// Constructs the QR decomposition of a square matrix A=QR. A is overwritten by R. Matrix Q is returned explicitly.
	template <typename Scalar, int N> __device__ __host__
	inline void xsqqr(Scalar *A, Scalar *Q)
	{
		Scalar d[N];
		Scalar c[N];
		int i,j,k;
		Scalar scale,sigma,sum,tau;
		scale = sigma = sum = tau =0.0;
		i=j=k=0;
		for (int k =0;k < N; k++)
		{
			for (i=k;i<N;i++)
				if (scale<abs(A[i+N*k]))
					scale=abs(A[i+N*k]);
			if (scale == 0.0) //Singular case.
				c[k]=d[k]=0.0;
			else 
			{
				for (i=k;i<N;i++)
					A[i+N*k] /= scale;
				for (sum=0.0,i=k;i<N;i++)
					sum += A[i+N*k]*A[i+N*k];
				sigma=A[k+N*k] > 0.0 ? sqrt(sum) : - sqrt(sum);
				A[k+N*k] += sigma;
				c[k]=sigma*A[k+N*k];
				d[k] = -scale*sigma;
				for (j=k+1;j<N;j++)
				{
					for (sum=0.0,i=k;i<N;i++) 
						sum += A[i+N*k]*A[i+N*j];
					tau=sum/c[k];
					for (i=k;i<N;i++) 
						A[i+N*j] -= tau*A[i+N*k];
				}
			}
		}
		d[N-1]*=-1;
		// Optionally get Q^T Matrix explicitly
		if (Q)
		{
			for (i=0;i<N;i++) {
				for (j=0;j<N;j++) Q[i+N*j]=0.0;
				Q[i+N*i]=1.0;
			}
			for (k=0;k<N-1;k++)
				if (c[k] != 0.0)
					for (j=0;j<N;j++)
					{
						sum=0.0;
						for (i=k;i<N;i++)
							sum += A[i+N*k]*Q[j+N*i];
						sum /= c[k];
						for (i=k;i<N;i++)
							Q[j+N*i] -= sum*A[i+N*k];
					}
		}
		// Get R Matrix explicitly
		Scalar *R=A;
		for (i =0; i < N; i++)
			for (j=0; j <N; j++)
				if (i == j)
					R[i+N*i] = d[i];
				else if ( i < j)
					R[i+N*j] = A[i+N*j];
				else
					R[i+N*j]=0;
	}

	/// Solve upper right triangular matrix for x (backward substitution)
	template <typename Scalar, int N> __device__ __host__
	inline void xutsolve(const Scalar *A, const Scalar b[N], Scalar x[N])
	{
		x[N-1] = b[N-1]/A[(N-1)*N + (N-1)];
		for (int i=(N-2); i>=0; i--)
		{
			x[i] = b[i];        
			for (int j=i+1; j<N; j++)
				x[i] -= A[(j)*N + i]*x[j];
			x[i] = x[i] / A[(i)*N + i];
		}
	}

	/// Inverse of upper right triangular matrix
	template <typename Scalar, int N> __device__ __host__
	inline void xutinv(const Scalar *U, Scalar *Uinv)
	{
		Scalar unit[N];
		for (int i=0;i<N;i++)
			unit[i]=0;
		for (int i=0;i<N;i++)
		{
			unit[i]=1;
			xutsolve<Scalar,N>(U,unit,Uinv+i*N);
			unit[i]=0;
		}
	}

	/// Solve Ax=b with regular matrix A=QR using QR decomposition Rx=Qt*b
	template <typename Scalar, int N> __device__ __host__
	inline void xsqqrsolve(const Scalar *Q, const Scalar *R, const Scalar b[N], Scalar x[N])
	{
		Scalar Qtb[N];
		for (int j=0;j<N;j++)
		{
			Scalar sum=0;
			for (int i=0;i<N;i++)
				sum+=b[i]*Q[i+N*j];
			Qtb[j]=sum;
		}
		xutsolve<Scalar,N>(R,Qtb,x);
	}

	/// Compute inverse using QR decomposition
	template <typename Scalar,int N> __device__ __host__
	inline void xgeinv(Scalar *A, Scalar *Ainv)
	{
		Scalar Q[N*N],R[N*N];
		for (int i=0;i<N*N;i++)
			R[i]=A[i];
		xsqqr<Scalar,N>(R,Q);
		Scalar unit[N];
		for (int i=0;i<N;i++)
			unit[i]=0;
		for (int i=0;i<N;i++)
		{
			unit[i]=1;
			xsqqrsolve<Scalar,N>(Q,R,unit,Ainv+i*N); // 2do optimize Qtb
			unit[i]=0;
		}
	}
	
} // namespace culaut

#ifdef udef__device__
#undef __device__
#endif
#ifdef udef__host__
#undef __host__
#endif

#endif // __xgeinv_hxx

