#ifndef __culaut_hxx
#define __culaut_hxx
// Cuda Linear Algebra Utility (low level version)
#ifndef __device__
#define __device__
#define udef__device__
#endif
#ifndef __host__
#define __host__
#define udef__host__
#endif

namespace culaut {
	
	/// Tiny class for constant size arrays. Use only though Vector/Matrix/MatrixNM.
	template <typename T, int N> struct Array {
		T v[N];
		Array(T* copy_from=0x0) {if (copy_from) array_copy(copy_from); }
		template <typename T2> void array_copy(T2* _array) { for (int i=0;i<N;i++) v[i]=(T)_array[i]; }
		void array_set(T value) { for (int i=0;i<N;i++) v[i]=value; }
	};

	/// Constant Size Vector
	template <typename T, int N> struct Vector : public Array<T,N> {
		Vector(T* _array=0x0) : Array<T,N>(_array) {}
		__device__ __host__ inline operator T*() {return this->v;}
		template <typename T2> __device__ __host__ inline static Vector<T,N> from(T2* _array) {
			Vector<T,N> ret;
			ret.array_copy(_array);
			return ret;
		}
	};

	/// Constant Size  Matrix
	template <typename T, int N> struct Matrix : public Array<T,N*N> {
		__device__ __host__ inline T& a(int i, int j) { return this->v[j*N+i]; }
		Matrix(T* _array=0x0) : Array<T,N*N>(_array) {}
		__device__ __host__ inline operator T*() {return this->v;}
		template <typename T2> __device__ __host__ inline
			static Matrix<T,N> from(T2* _array) {
			Matrix<T,N> ret;
			ret.array_copy(_array);
			return ret;
		}
	};

	/// Constant Size M*N Matrix
	template <typename T, int M, int N> struct MatrixMN : public Array<T,M*N> {
		__device__ __host__ inline T& a(int i, int j) { return this->v[j*M+i]; }
		MatrixMN(T* _array=0x0) : Array<T,M*N>(_array) {}
		__device__ __host__ inline operator T*() {return this->v;}
		template <typename T2> __device__ __host__ inline static MatrixMN<T,M,N> from(T2* _array) {
			MatrixMN<T,M,N> ret;
			ret.array_copy(_array);
			return ret;
		}
	};

	/// Set all element of a vector to a constant
	template <typename T, int N> __device__ __host__ inline
	void xvset(T *v, T c) {
		for (int i=0;i<N;i++) v[i]=c;
	}

	/// Set a vector by three values
	template <typename T> __device__ __host__ inline
	void xvset3(T v[3], T v0, T v1, T v2) {
		v[0]=v0; v[1]=v1; v[2]=v2;
	}

	/// Set a vector by four values
	template <typename T> __device__ __host__ inline
	void xvset4(T v[4], T v0, T v1, T v2, T v3) {
		v[0]=v0; v[1]=v1; v[2]=v2; v[3]=v3;
	}

	/// out=a*x+y
	//template <typename T1, typename T2, int N> __device__ __host__ inline
	//void xaxpy(T1 *x, float a, float y, T2 *out) {
	//	for (int i=0;i<N;i++) out[i]=(T2)(x[i]*a)+y;
	//}

	/// dst=a*x+b (please note the difference to the BLAS instruction)
	template <typename T1, typename T2, int N> __device__ __host__ inline
	void xvaxpy(T1 dst, T2 *a, T2 *x, T2 *y) {
		for (int i=0;i<N;i++) dst[i]=(T2)(a[i]*x[i])+y[i];
	}

	/// a=b
	template <typename T1, typename T2, int N> __device__ __host__ inline
	void xvcpy(T1 *dst, const T2 *src) {
		for (int i=0;i<N;i++) dst[i]=(T1)src[i];
	}

	// linear combination dst=src1*w1+src2*w2
	template <typename T1, typename T2, int N> __device__ __host__ inline
	void xvlincomb(T1 *dst, const T2 *src1, float w1, const T2 *src2, float w2) {
		for (int i=0;i<N;i++) dst[i]=(T1)(w1*src1[i]+w2*src2[i]);
	}

	/// a*=b
	template <typename T, int N> __device__ __host__ inline
	void xvscale(T *a, T b) {
		for (int i=0;i<N;i++) a[i]*=b;
	}

	/// a+=b
	template <typename T, int N> __device__ __host__ inline
	void xvadd(T *a, T *b) {
		for (int i=0;i<N;i++) a[i]+=b[i];
	}

	/// dst=a+b
	template <typename T1, typename T2, int N> __device__ __host__ inline
	void xvadd(T1 *dst, T2 *a, T2 *b) {
		for (int i=0;i<N;i++) dst[i]=(T1)(a[i]+b[i]);
	}

	/// a-=b
	template <typename T, int N> __device__ __host__ inline
	void xvsub(T *a, T *b) {
		for (int i=0;i<N;i++) a[i]-=b[i];
	}

	/// dst=a-b
	template <typename T1, typename T2, int N> __device__ __host__ inline
	void xvsub(T1 *dst, T2 *a, T2 *b) {
		for (int i=0;i<N;i++) dst[i]=(T1)(a[i]-b[i]);
	}

	/// a*=b
	template <typename T, int N> __device__ __host__ inline
	void xvmul(T *a, T *b) {
		for (int i=0;i<N;i++) a[i]*=b[i];
	}

	/// dst=a*b
	template <typename T1, typename T2, int N> __device__ __host__ inline
	void xvmul(T1 *dst, T2 *a, T2 *b) {
		for (int i=0;i<N;i++) dst[i]=(T1)(a[i]*b[i]);
	}

	/// a/=b
	template <typename T, int N> __device__ __host__ inline
	void xvdiv(T *a, T *b) {
		for (int i=0;i<N;i++) a[i]/=b[i];
	}

	/// dst=a/b
	template <typename T1, typename T2, int N> __device__ __host__ inline
	void xvdiv(T1 *dst, T2 *a, T2 *b) {
		for (int i=0;i<N;i++) dst[i]=(T1)(a[i]/b[i]);
	}

	/// Dot product
	template <typename T, int N> __device__ __host__ inline
	T xvdot(const T *a, const T *b)
	{
		T ret=0;
		for (int i=0;i<N;i++)
			ret+=a[i]*b[i];
		return ret;
	}

	/// Cross product
	template <typename T> __device__ __host__ inline
	void xvcross(const T a[3], const T b[3], T c[3])
	{
		c[0]=a[1]*b[2]-a[2]*b[1];
		c[1]=a[2]*b[0]-a[0]*b[2];
		c[2]=a[0]*b[1]-a[1]*b[0];
	}

	/// Euclidian norm
	template <typename T, int N> __device__ __host__ inline
	T xvnorm2(T *a) {
		return sqrt(xvdot<T,N>(a,a));
	}

	/// Maximum norm
	template <typename T, int N> __device__ __host__ inline
	T xvmax(T *a)
	{
		T ret=a[0];
		for (int i=1;i<N;i++)
			if (a[i]>ret) ret=a[i];
		return ret;
	}

	/// Minimum
	template <typename T, int N> __device__ __host__ inline
	T xvmin(T *a)
	{
		T ret=a[0];
		for (int i=1;i<N;i++)
			if (a[i]<ret) ret=a[i];
		return ret;
	}

	/// Normalize
	template <typename T, int N> __device__ __host__ inline
	void xvnormalize2(T *a) {
		T norm=xvnorm2<T,N>(a);
		if (norm!=0) xvscale<T,N>(a,1.0f/(float)norm);
	}
	
	/// Normalize by l_infty
	template <typename T, int N> __device__ __host__ inline
	void xvnormalize_max(T *a) {
		T norm=xvmax<T,N>(a);
		if (norm!=0) xvscale<T,N>(a,(float)1.0/norm);
	}

	/// Normal equality class for projective objects. N is vector length (e.g. 3 for RP2)
	template <typename T, int N> __device__ __host__ inline
	void xvdehomogenize(T *a)
	{
		// Handle points (almost) at infinity specifically
		if (a[N-1]<1e-12 && a[N-1]>-1e-12)
		{
			xvnormalize_max<T,N>(a);
			if (a[N-1]<1e-12 && a[N-1]>-1e-12)
			{
				a[N-1]=0;
				return;
			}
			else // Input point is inappropriately scaled!
				;// If you get here, you have ignored numerical considerations. 
		}
		xvscale<T,N>(a,(float)1.0/a[N-1]);
	}

	/// Euclidian distance
	template <typename T, int N> __device__ __host__ inline
	T xvdistance2(T *a, T *b)
	{
		T ret=0;
		for (int i=0;i<N;i++)
			ret+=(a[i]-b[i])*(a[i]-b[i]);
		return sqrt(ret);
	}

	/// Determinant (unstable version) for 3x3 matrices
	template <typename T> __device__ __host__ inline
	T xvdet3(T a[3], T b[3], T c[3])
	{
		return
			+a[0]*(b[1]*c[2]-b[2]*c[1])
			+b[0]*(c[1]*a[2]-c[2]*a[1])
			+c[0]*(a[1]*b[2]-a[2]*b[1]);
	}

	/// Determinant (unstable version) for 3x3 matrices
	template <typename T> __device__ __host__ inline
	T xgedet3(T a[3*3]) {
		return xvdet3(a,a+3,a+6);
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
	
	/// General matrix multiplication of transpose: (OxM)^T times  OxN result is MxN (exmple: xgemmT<double,3,4,1>(P, l, E) )
	template <typename T, int O, int M, int N> __device__ __host__ inline
	void xgemmT(const T* At, const T* B, T* C)
	{
		for (int i=0;i<M;i++)
			for (int j=0;j<N;j++)
			{
				T sum=0;
				for (int s=0;s<O;s++)
					sum+=At[s+i*M]*B[j*O+s];
				C[j*M+i]=sum;
			}
	}

	/// General matrix vector multiplication MxO times Ox1 result is Mx1
	template <typename T, int M, int O> __device__ __host__ inline
	void xgemv(T* A, T* B, T* C)
	{
		xgemm<T,M,O,1>(A,B,C);
	}
	
	/// Fill 3x3 matrix with values
	template <typename T> __device__ __host__ inline
	void xgeset3(T m[3*3],	T a00=1, T a01=0, T a02=0,
							T a10=0, T a11=1, T a12=0,
							T a20=0, T a21=0, T a22=1)
	{
		m[0*3+0]=a00; m[1*3+0]=a01;	m[2*3+0]=a02;
		m[0*3+1]=a10; m[1*3+1]=a11;	m[2*3+1]=a12;
		m[0*3+2]=a20; m[1*3+2]=a21;	m[2*3+2]=a22;
	}

	/// Fill 4x4 (upper left sub-) matrix with values
	template <typename T> __device__ __host__ inline
	void xgeset4(T m[4*4],	T a00=1, T a01=0, T a02=0, T a03=0,
							T a10=0, T a11=1, T a12=0, T a13=0,
							T a20=0, T a21=0, T a22=1, T a23=0,
							T a30=0, T a31=0, T a32=0, T a33=1)
	{
		m[0+4*0]=a00;	m[0+4*1]=a01;	m[0+4*2]=a02;	m[0+4*3]=a03;
		m[1+4*0]=a10;	m[1+4*1]=a11;	m[1+4*2]=a12;	m[1+4*3]=a13;
		m[2+4*0]=a20;	m[2+4*1]=a21;	m[2+4*2]=a22;	m[2+4*3]=a23;
		m[3+4*0]=a30;	m[3+4*1]=a31;	m[3+4*2]=a32;	m[3+4*3]=a33;
	}
	
} // namespace culaut

#ifdef udef__device__
#undef __device__
#endif
#ifdef udef__host__
#undef __host__
#endif

#endif // __culaut_hxx
