// Created by A. Aichert on Tue Oct 4th 2016
// Contains types and functions to test for intersection of ray and axis aligned bounding box

#ifndef __ray_box_intersection_hxx
#define __ray_box_intersection_hxx

#ifndef __device__
#define __device__
#define udef__device__
#endif
#ifndef __host__
#define __host__
#define udef__host__
#endif

#include "vr_cuda.hxx"

namespace RayBoxIntersection
{
	
	template <typename T>
	struct Ray {
		T origin[3];
		T direction[3];
	};
	
	template <typename T>
	struct Intersections {
		T x_min[3];
		T x_max[3];
	};
	
	/// Test for intersection of ray with axis aligned box. Intersections are returned as optional out_min/max.
	template <typename T> __device__ __host__ inline
	bool ray_box_intersection(const Ray<T>& ray, const AxisAlignedBox<T>& box, Intersections<T>& out)
	{
		return ray_box_intersection<T>(ray.origin, ray.direction, box.aabb_min, box.aabb_max, out.x_min, out.x_max);
	}

	/// Test for intersection of ray (origin, direction) with axis aligned box (aabb_min,aabb_max). Intersections are returned as optional out_min/max.
	template <typename T> __device__ __host__ inline
	bool ray_box_intersection(T origin[3], T direction[3], T aabb_min[3], T aabb_max[3], T out_min[3]=0x0, T out_max[3]=0x0)
	{
		// multipliers for direction min and max ("close" and "far" if positive)
		T t_min[3];
		T t_max[3];

		// figure out intersections for the x-, y- and z-planes
		{
			T invdir;
			invdir=1.0f/direction[0];
			t_min[0] = ((invdir>0?aabb_min[0]:aabb_max[0]) - origin[0]) * invdir;
			t_max[0] = ((invdir<0?aabb_min[0]:aabb_max[0]) - origin[0]) * invdir;
			invdir=1.0f/direction[1];
			t_min[1] = ((invdir>0?aabb_min[1]:aabb_max[1]) - origin[1]) * invdir;
			t_max[1] = ((invdir<0?aabb_min[1]:aabb_max[1]) - origin[1]) * invdir;
			invdir=1.0f/direction[2];
			t_min[2] = ((invdir>0?aabb_min[2]:aabb_max[2]) - origin[2]) * invdir;
			t_max[2] = ((invdir<0?aabb_min[2]:aabb_max[2]) - origin[2]) * invdir;
		}

		// Check for contradictions (ray intersections do not touch box)
		if ((t_min[0] > t_max[1]) || (t_min[1] > t_max[0])) return false;
		// Keep track of tightest fit.
		if (t_min[1] > t_min[0]) t_min[0] = t_min[1]; 
		if (t_max[1] < t_max[0]) t_max[0] = t_max[1]; 
	  
		// Again check for contradictions
		if ((t_min[0] > t_max[2]) || (t_min[2] > t_max[0])) return false;
		// Again keep track of tightest fit.
		if (t_min[2] > t_min[0]) t_min[0] = t_min[2];
		if (t_max[2] < t_max[0]) t_max[0] = t_max[2];
	 
		// Optional: intersection output
		if (out_min)
		{
			out_min[0]=origin[0]+direction[0]*t_min[0];
			out_min[1]=origin[1]+direction[1]*t_min[0];
			out_min[2]=origin[2]+direction[2]*t_min[0];
		}
		if (out_max)
		{
			out_max[0]=origin[0]+direction[0]*t_max[0];
			out_max[1]=origin[1]+direction[1]*t_max[0];
			out_max[2]=origin[2]+direction[2]*t_max[0];
		}

		return true;
	} 

} // namespace RayBoxIntersection

#ifdef udef__device__
#undef __device__
#endif
#ifdef udef__host__
#undef __host__
#endif

#endif // __ray_box_intersection_hxx
