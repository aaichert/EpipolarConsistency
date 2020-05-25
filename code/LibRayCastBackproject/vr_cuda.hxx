#ifndef __vr_cuda_hxx
#define __vr_cuda_hxx

#ifndef __device__
#define __device__
#define udef__device__
#endif
#ifndef __host__
#define __host__
#define udef__host__
#endif

template <typename T>
struct AxisAlignedBox {
	T aabb_min[3];
	T aabb_max[3];
};

//struct TransferFunctionRamp {
//	float in_min;
//	float in_max;
//	float out_max;
//
//	__device__ __host__ inline
//	float operator()(float in)
//	{
//		if (in<=in_min) return 0;
//		if (in>=in_max) return out_max;
//		return out_max*(in-in_min)/(in_max-in_min);
//	}
//};

#ifdef udef__device__
#undef __device__
#endif
#ifdef udef__host__
#undef __host__
#endif

#endif // __vr_cuda_hxx
