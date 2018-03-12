// Created by A. Aichert on Dec 20th 2017

#ifndef __line_perspectivity_hxx
#define __line_perspectivity_hxx

#ifndef __device__
#define __device__
#define udef__device__
#endif
#ifndef __host__
#define __host__
#define udef__host__
#endif

/// Perspectivity on a Line l=H^T*l_prime in an image transformed by x_prime=H*x
struct LinePerspectivity {
	float a=1.f;
	float b=0.f;
	float c=0.f;
	float d=1.f;

// We use Eigen only in c++ code, not when compiled for CUDA
#ifdef __real_projective_space_types_defined
	/// Compute line coordinate of a 2D point x on the line l
	static Geometry::RP2Point project_to_plane(double t, Geometry::RP2Line& l) {
			return Geometry::RP2Point(-l[2]*l[0]+t*l[1],-l[2]*l[1]-t*l[0],1.0);
	}

	/// Compute line coordinate of a 2D point x on the line l
	static double project_to_line(const Geometry::RP2Point& x, const Geometry::RP2Line& l) {
			return l[1]*x[0]/x[2] -l[0]*x[1]/x[2];
	}

	/// Line distance (is zero for point x on line l)
	static double line_distance(double u, double v, Geometry::RP2Line& l) {
		return l[0]*u +l[1]*v+l[2];
	}

	/// Mapping of line coordinates on a line in a projectively transforming image.
	LinePerspectivity(const Geometry::RP2Homography& H, const Geometry::RP2Line& l) {
		a=(float)(       H(0,0)*l(1)-H(1,0)*l(0)                  );
		b=(float)(H(0,2)                         -H(0,0)*l(0)*l(2));
		c=(float)(       H(2,0)*l(1)-H(2,1)*l(0)                  );
		d=(float)(H(2,2)-H(2,0)*l(0)*l(2)        -H(2,1)*l(1)*l(2));
		if (a*d - b*c < 0) {
			a*=-1;
			b*=-1;
		}
	}

	/// 2D rigid transformation which brings closest point on line to origin and rotates line into u-axis.
	static Geometry::RP2Homography mapping_to_line_coordinates(const Geometry::RP2Line& l) {
		Geometry::RP2Homography Hlu;
		Hlu << 
			l[1],-l[0],  0.,
			l[0], l[1],l[2],
			  0.,   0.,  1.;
		return Hlu;
	}
#endif // __real_projective_space_types_defined

	/// Initialize with identity or from raw memory
	__device__ __host__ LinePerspectivity(float *v=0x0) : a(v?v[0]:1.f), b(v?v[1]:.0f), c(v?v[2]:.0f), d(v?v[3]:1.f) {}
	
	/// Perspective transformation of line coordinate
	__device__ __host__ float transform (float t)       const {
		return (a*t+b)/(c*t+d);
	}

	/// Inverse perspective transformation of line coordinate
	__device__ __host__ float inverse   (float t_prime) const {
		return (d*t_prime-b)/(-c*t_prime+a);
	}

	/// Derivative of transform(...)
	__device__ __host__ float derivative(float t)       const {
		return (a*d-b*c) / (c*c*t*t +2*c*d*t +d*d);
	}

};

/// Mixed bad of all important information to compute FBCC in cone-beam projections.
struct FBCC_weighting_info {
	LinePerspectivity phi; //< Transformation from line coords on l_kappa to virtual detector u-axis
	float t_prime_ak;      //< Closest point to epipolar line in virtual detector u-coordinates
	float d_l_kappa_C_sq;  //< Squared pixel distance from source to epipolar line
	float dummy0,dummy1;
};

#ifdef udef__device__
#undef __device__
#endif
#ifdef udef__host__
#undef __host__
#endif

#endif // __line_perspectivity_hxx
