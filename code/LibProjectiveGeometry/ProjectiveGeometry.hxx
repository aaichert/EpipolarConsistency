#ifndef __projective_geometry_hxx
#define __projective_geometry_hxx
// Created by aaichert on Wed Sept 3rd 2014
// Types for projective two- and three-space including join and meet operations for points, lines and planes.

#include <Eigen/Core>

#include <limits>

namespace Geometry
{
	/// typedefs to increase readability
	#ifndef __real_projective_space_types_defined
	#define __real_projective_space_types_defined
	typedef Eigen::Vector3d           RP2Point;            // Homogeneous coordinates of a real 2D point.
	typedef Eigen::Vector3d           RP2Line;             // Real 2D line (a,b,c) of points (x,y) with ax+by+c=0.
	typedef Eigen::Vector4d           RP3Point;            // Homogeneous coordinates of a real 3D point.
	typedef Eigen::Matrix<double,6,1> RP3Line;             // Plücker coordinates of a 3D line. 
	typedef Eigen::Vector4d           RP3Plane;            // Plane (a,b,c,d) of points (x,y,z) with ax+by+cz+d=0.
	typedef Eigen::Matrix3d           RP2Homography;       // Projective transformation of real projective two-space.
	typedef Eigen::Matrix4d           RP3Homography;       // Projective transformation of real projective three-space.
	typedef Eigen::Matrix<double,3,4> ProjectionMatrix;
	const   Eigen::Vector3d           infinity2(0,0,1);    // Line at infinity.
	const   Eigen::Vector4d           infinity3(0,0,0,1);  // Plane at infinity.
	const   Eigen::Vector3d           origin2(0,0,1);      // Origin of two space.
	const   Eigen::Vector4d           origin3(0,0,0,1);    // Origin of three space.
	// Construction from euclidian points and directions:
	inline  RP2Point finitePoint  (const Eigen::Vector2d& position)
		{ auto D=RP2Point::Ones().eval(); D.head(2)=position; return D;}
	inline  RP3Point finitePoint  (const Eigen::Vector3d& position)
		{ auto D=RP3Point::Ones().eval(); D.head(3)=position; return D;}
	inline  RP2Point infinitePoint(const Eigen::Vector2d& direction)
		{ auto D=RP2Point::Zero().eval(); D.head(2)=direction.normalized(); return D;}
	inline  RP3Point infinitePoint(const Eigen::Vector3d& direction)
		{ auto D=RP3Point::Zero().eval(); D.head(3)=direction.normalized(); return D;}
	
	/// Divide by homogeneous component, except for infinite points. Those, normalize to unit length. See also: dehomogenized(...)
	inline bool dehomogenize(RP2Point& x)
	{
		if ( x(2)>1e-11 || x(2)<-1e-11)
		{
			// finite point
			x/=x(2);
			return true;
		}
		else
		{
			// infinite points
			x(2)=0;
			x/=x.head(2).norm();
			return false;
		}
	}

	/// Divide by homogeneous component, except for infinite points. Those, normalize to unit length. See also: dehomogenize(...)
	inline RP2Point dehomogenized(RP2Point x) {dehomogenize(x); return x;}

	/// Convert to euclidian point. For sensible result, see also: allfinite(...).
	inline Eigen::Vector2d euclidian2(RP2Point x)
	{
		if (!dehomogenize(x)) x.setConstant(std::numeric_limits<double>::infinity());
		return x.head(2);
	}

	/// Component-wise check for inf and nan.
	template <typename Vector> bool allfinite(const Vector& x)
	{
		bool ret=true;
		for (int i=0;i<(int)x.cols();i++)
			ret&=std::isfinite(x[i]);
		return ret;
	}

	/// Divide by homogeneous component, except for infinite points. Those, normalize to unit length. See also: dehomogenized(...)
	inline bool dehomogenize(RP3Point& X)
	{
		if ( X(3)>1e-12 || X(3)<-1e-12)
		{
			// finite point
			X/=X(3);
			return true;
		}
		else
		{
			// infinite points
			X(3)=0;
			X/=X.head(3).norm();
			return false;
		}
	}

	/// Divide by homogeneous component, except for infinite points. Those, normalize to unit length. See also: dehomogenize(...)
	inline RP3Point dehomogenized(RP3Point X) {dehomogenize(X); return X;}

	/// Convert to euclidian point. For sensible result, see also: allfinite(...).
	inline Eigen::Vector3d euclidian3(RP3Point x)
	{
		if (!dehomogenize(x)) x.setConstant(std::numeric_limits<double>::infinity());
		return x.head(3);
	}

	/// Convert to Hessian normal form. See also: normalized(...)
	inline bool normalize(RP2Line& l)
	{
		double norm=l.head(2).norm();
		if ( norm>1e-12 || norm<-1e-12)
		{
			l/=norm;
			return true;
		}
		else
		{
			l=infinity2;
			return false;
		}
	}

	/// Convert to Hessian normal form. See also: normalize(...)
	inline RP2Line normalized(RP2Line l) {normalize(l);return l;}

	/// Convert to Hessian normal form. See also: normalized(...)
	inline bool normalize(RP3Plane& E)
	{
		double norm=E.head(3).norm();
		if ( norm>1e-12 || norm<-1e-12)
		{
			E/=norm;
			return true;
		}
		else
		{
			E=infinity3;
			return false;
		}
	}

	/// Convert to Hessian normal form. See also: normalize(...)
	inline RP3Plane normalized(RP3Plane E) {normalize(E);return E;}
	
	#endif // __real_projective_space_types_defined

	#ifndef __pi_defined
	#define __pi_defined
	const double Pi=3.14159265358979323846264338327950288419716939937510582;
    #endif // __pi_defined
	
	//////////
	// RP^2
	//////////

	/// Join two points to form a line (orientation: from x0 to x1 i.e. cross(x1,x0) )
	inline RP2Line  join(const RP2Point& x1, const RP2Point& x0) {return x1.cross(x0);}

	/// Meet of two lines to form a point (positive iff they point in the same direction +/- 90 degrees)
	inline RP2Point meet(const RP2Line&  l1, const RP2Line&  l0) {return l1.cross(l0);}

	/// Convert homogeneous line representation to angle y-intercept (intercept greater zero)
	inline Eigen::Vector2d lineToAngleIntercept(const RP2Line& l)
	{
		double alpha=std::atan2(l(1),l(0))-Pi*0.5; // angle
		while (alpha<-Pi) alpha+=2.0*Pi;
		double t=-l(2)/std::sqrt(l(0)*l(0)+l(1)*l(1)); // intercept
		return Eigen::Vector2d(alpha,t);
	}

	/// Change of coordinate system from lower left corner to image center.
	inline void lineRelativeToCenter(double& alpha, double& t, int n_x, int n_y)
	{
		// Compensate for location of origin (lower left corner versus center of image)
		t-=0.5*n_x*std::cos(alpha+Pi*0.5)+0.5*n_y*std::sin(alpha+Pi*0.5);
	}

	/// Convert angle intercept line representation to homogeneous
	inline RP2Line line(double alpha, double t)
	{
		RP2Line l;
		l(0)=std::cos(alpha+Pi*0.5); // normal
		l(1)=std::sin(alpha+Pi*0.5); // normal
		l(2)=-t;
		return l;
	}
	
	//////////
	// RP^3
	//////////

	/// Join two points to form a line
	inline RP3Line join_pluecker(const RP3Point& A, const RP3Point& B)
	{
		RP3Line L;
		L<<
			A(0)*B(1)-A(1)*B(0),
			A(0)*B(2)-A(2)*B(0),
			A(0)*B(3)-A(3)*B(0),
			A(1)*B(2)-A(2)*B(1),
			A(1)*B(3)-A(3)*B(1),
			A(2)*B(3)-A(3)*B(2);
		return L;
	}

	/// Meet two planes to form a line
	inline RP3Line meet_pluecker(const RP3Plane& A, const RP3Plane& B)
	{
		RP3Line L;
		L<<
			A(2)*B(3)-A(3)*B(2),
			A(3)*B(1)-A(1)*B(3),
			A(1)*B(2)-A(2)*B(1),
			A(0)*B(3)-A(3)*B(0),
			A(2)*B(0)-A(0)*B(2),
			A(0)*B(1)-A(1)*B(0);
		return L;
	}

	/// Join a line and a point to form a plane
	inline RP3Plane join_pluecker(const RP3Line& L, const RP3Point& X)
	{
		return RP3Plane(
				             + X(1)*L(5) - X(2)*L(4) + X(3)*L(3),
				 - X(0)*L(5)             + X(2)*L(2) - X(3)*L(1),
				 + X(0)*L(4) - X(1)*L(2)             + X(3)*L(0),
				 - X(0)*L(3) + X(1)*L(1) - X(2)*L(0)
			);
	}

	/// Meet a line and a plane to form a point
	inline RP3Point meet_pluecker(const RP3Line& L, const RP3Plane& P)
	{
		return RP3Point(
				             - P(1)*L(0) - P(2)*L(1) - P(3)*L(2),
				 + P(0)*L(0)             - P(2)*L(3) - P(3)*L(4),
				 + P(0)*L(1) + P(1)*L(3)             - P(3)*L(5),
				 + P(0)*L(2) + P(1)*L(4) + P(2)*L(5)
			);
	}
	
	////////////////////////////////////////
	// Geometric Interpretation
	////////////////////////////////////////

	/// The moment of a line (plane orthogonal to line through origin)
	inline Eigen::Vector3d pluecker_direction(const RP3Line& L)
	{
		return Eigen::Vector3d(-L[2], -L[4], -L[5]);
	}

	/// Direction of a line
	inline Eigen::Vector3d pluecker_moment(const RP3Line& L)
	{
		return Eigen::Vector3d(L[3], -L[1], L[0]);
	}

	/// Closest point on line L to the origin.
	inline RP3Point pluecker_closest_point_to_origin(const RP3Line& L)
	{
		return RP3Point(
			L[4]*L[0]+L[1]*L[5],
			-L[0]*L[2]+L[3]*L[5],
			-L[1]*L[2]-L[3]*L[4],
			-L[2]*L[2]-L[4]*L[4]-L[5]*L[5]
			);
	}

	/// Distance of a line to the origin
	inline double pluecker_distance_to_origin(const RP3Line& L)
	{
		return pluecker_moment(L).norm()/pluecker_direction(L).norm();
	}

	/// Compute the closest point on the line L to a point X
	inline RP3Point pluecker_closest_to_point(const RP3Line& L, RP3Point X)
	{
		auto direction=pluecker_direction(L);
		auto plane_through_X_orthogonal_to_L=RP3Plane(direction[0],direction[1],direction[2],-direction.dot(euclidian3(X))).eval();
		auto closest_point_to_X_on_L=meet_pluecker(L,plane_through_X_orthogonal_to_L);
		return closest_point_to_X_on_L;

	}

	////////////////////
	// Pluecker Matrices
	////////////////////
	
	/// Anti-symmetric matrix for the join operation using dual Plücker coordinates
	inline Eigen::Matrix4d plueckerMatrixDual(const RP3Line& L)
	{
		Eigen::Matrix4d B;
		B << 
			    0 , + L(5), - L(4), + L(3),
			- L(5),     0 , + L(2), - L(1),
			+ L(4), - L(2),     0 , + L(0),
			- L(3), + L(1), - L(0),     0;
		return B;
	}

	/// Anti-symmetric matrix for the meet operation dual Plücker coordinates
	inline Eigen::Matrix4d plueckerMatrix(const RP3Line& L)
	{
		Eigen::Matrix4d B;
		B << 
				     0 , - L(0), - L(1), - L(2),
				 + L(0),     0 , - L(3), - L(4),
				 + L(1), + L(3),     0 , - L(5),
				 + L(2), + L(4), + L(5),     0;
		return B;
	}
	
	////////////////////
	// Projections
	////////////////////

	/// Sturm-style projection matrix for Plücker lines. Projectoin from Plücker coordinates directly to 2D lines.
	inline Eigen::Matrix<double,6,1> pluecker_projection_matrix(const ProjectionMatrix& P)
	{
		Eigen::Matrix<double,6,1> PL;
		PL << 
			P(1,0)*P(2,1)-P(1,1)*P(2,0),+P(1,0)*P(2,2)-P(1,2)*P(2,0),+P(1,0)*P(2,3)-P(1,3)*P(2,0),+P(1,1)*P(2,2)-P(1,2)*P(2,1),+P(1,1)*P(2,3)-P(1,3)*P(2,1),+P(1,2)*P(2,3)-P(1,3)*P(2,2),
			P(0,1)*P(2,0)-P(0,0)*P(2,1),-P(0,0)*P(2,2)+P(0,2)*P(2,0),-P(0,0)*P(2,3)+P(0,3)*P(2,0),-P(0,1)*P(2,2)+P(0,2)*P(2,1),-P(0,1)*P(2,3)+P(0,3)*P(2,1),-P(0,2)*P(2,3)+P(0,3)*P(2,2),
			P(0,0)*P(1,1)-P(0,1)*P(1,0),+P(0,0)*P(1,2)-P(0,2)*P(1,0),+P(0,0)*P(1,3)-P(0,3)*P(1,0),+P(0,1)*P(1,2)-P(0,2)*P(1,1),+P(0,1)*P(1,3)-P(0,3)*P(1,1),+P(0,2)*P(1,3)-P(0,3)*P(1,2);
		return PL;
	}
	
	/// Directly project 3D line in Plücker coordinates to 2D line.
	inline RP2Line pluecker_project(const RP3Line& L, const ProjectionMatrix& P)
	{
		return RP2Line(
			L[0]*(P(1,0)*P(2,1)-P(1,1)*P(2,0))+L[1]*(+P(1,0)*P(2,2)-P(1,2)*P(2,0))+L[2]*(+P(1,0)*P(2,3)-P(1,3)*P(2,0))+L[3]*(+P(1,1)*P(2,2)-P(1,2)*P(2,1))+L[4]*(+P(1,1)*P(2,3)-P(1,3)*P(2,1))+L[5]*(+P(1,2)*P(2,3)-P(1,3)*P(2,2)),
			L[0]*(P(0,1)*P(2,0)-P(0,0)*P(2,1))+L[1]*(-P(0,0)*P(2,2)+P(0,2)*P(2,0))+L[2]*(-P(0,0)*P(2,3)+P(0,3)*P(2,0))+L[3]*(-P(0,1)*P(2,2)+P(0,2)*P(2,1))+L[4]*(-P(0,1)*P(2,3)+P(0,3)*P(2,1))+L[5]*(-P(0,2)*P(2,3)+P(0,3)*P(2,2)),
			L[0]*(P(0,0)*P(1,1)-P(0,1)*P(1,0))+L[1]*(+P(0,0)*P(1,2)-P(0,2)*P(1,0))+L[2]*(+P(0,0)*P(1,3)-P(0,3)*P(1,0))+L[3]*(+P(0,1)*P(1,2)-P(0,2)*P(1,1))+L[4]*(+P(0,1)*P(1,3)-P(0,3)*P(1,1))+L[5]*(+P(0,2)*P(1,3)-P(0,3)*P(1,2)));
	}

	/// A mapping T from a 3D point to a plane E via central projection from C. T*X=meet(join(C,X),E)
	inline RP3Homography centralProjectionToPlane(const RP3Point& C, const RP3Point& E)
	{
		Eigen::Matrix4d P;
		P << 
		 + C[1]*E[1] + C[2]*E[2] + C[3]*E[3] , - C[0]*E[1]                         , - C[0]*E[2]                         , - C[0]*E[3]                        ,
		 - C[1]*E[0]                         , + C[0]*E[0] + C[2]*E[2] + C[3]*E[3] , - C[1]*E[2]                         , - C[1]*E[3]                        ,
		 - C[2]*E[0]                         , - C[2]*E[1]                         , + C[0]*E[0] + C[3]*E[3] + C[1]*E[1] , - C[2]*E[3]                        ,
		 - C[3]*E[0]                         , - C[3]*E[1]                         , - C[3]*E[2]                         , + C[0]*E[0] + C[1]*E[1] + C[2]*E[2];
		return P;
	}

	/////////////////////////
	// Useful homographies
	/////////////////////////

	/// Homogeneous 2D rotation
	inline RP2Homography Rotation(double alpha)
	{
		double ca=cos(alpha);
		double sa=sin(alpha);
		Eigen::Matrix3d R;
		R << 
			  ca, -sa, 0,
			  sa,  ca, 0,
			   0,   0, 1;
		return R;
	}

	/// Homogeneous 2D rigid transformaion
	inline RP2Homography Rigid(double alpha, double tu, double tv)
	{
		Eigen::Matrix3d R;
		double ca=cos(alpha);
		double sa=sin(alpha);
		R << 
			  ca, -sa, tu,
			  sa,  ca, tv,
			   0,   0, 1;
		return R;
	}

	/// Homogeneous 2D translation
	inline RP2Homography Translation(double tu, double tv)
	{
		Eigen::Matrix3d T;
		T << 
			  1, 0, tu,
			  0, 1, tv,
			  0, 0, 1;
		return T;
	}

	/// Homogeneous 2D scaling
	inline RP2Homography Scale(double su, double sv)
	{
		Eigen::Matrix3d S;
		S << 
			  su, 0,  0,
			  0,  sv, 0,
			  0,  0,  1;
		return S;
	}

	/// Homogeneous rotation about X-axis
	inline RP3Homography RotationX(double alpha)
	{
		double ca=cos(alpha);
		double sa=sin(alpha);
		Eigen::Matrix4d R;
		R << 
			 1,  0,   0, 0,
			 0, ca, -sa, 0,
			 0, sa,  ca, 0,
			 0,  0,   0, 1;
		return R;
	}

	/// Homogeneous rotation about Y-axis
	inline RP3Homography RotationY(double alpha)
	{
		double ca=cos(alpha);
		double sa=sin(alpha);
		Eigen::Matrix4d R;
		R << 
			 ca,  0, sa, 0,
			  0,  1,  0, 0,
			-sa,  0, ca, 0,
			  0,  0,  0, 1;
		return R;
	}
	
	/// Homogeneous rotation about Z-axis
	inline RP3Homography RotationZ(double alpha)
	{
		double ca=cos(alpha);
		double sa=sin(alpha);
		Eigen::Matrix4d R;
		R << 
		 	 ca, -sa, 0, 0,	
			 sa,  ca, 0, 0,
			  0,   0, 1, 0,
			  0,   0, 0, 1;
		return R;
	}

	/// Homogeneous translation
	inline RP3Homography Translation(double tX,double tY,double tZ)
	{
		Eigen::Matrix4d T;
		T << 
		 	 1, 0, 0, tX,
			 0, 1, 0, tY,
			 0, 0, 1, tZ,
			 0, 0, 0, 1;
		return T;
	}
	
	/// Homogeneous translation
	inline RP3Homography Translation(const Eigen::Vector3d& t)
	{
		Eigen::Matrix4d T;
		T << 
		 	 1, 0, 0, t[0],
			 0, 1, 0, t[1],
			 0, 0, 1, t[2],
			 0, 0, 0, 1;
		return T;
	}

	/// Homogeneous 3D scaling
	inline RP3Homography Scale(double sx, double sy, double sz)
	{
		Eigen::Matrix4d S;
		S << 
		 	 sx, 0,  0,  0,
			 0,  sy, 0,  0,
			 0,  0,  sz, 0,
			 0,  0,  0,  1;
		return S;
	}

	/// Homogeneous 3D scaling
	inline RP3Homography Scale(const Eigen::Vector3d& s)
	{
		Eigen::Matrix4d S;
		S << 
		 	 s(0), 0,    0,    0,
			 0,    s(1), 0,    0,
			 0,    0,    s[2], 0,
			 0,    0,    0,    1;
		return S;
	}

	/////////////////////////
	// Utilities (metric)
	/////////////////////////

	/// Compute angle relative to zero-plane E0 and 90° plane E1. Input assumed to be in HNF.
	inline double plane_angle_in_pencil(const RP3Plane& E, const RP3Plane& E0, const RP3Plane& E90)
	{
		double sina=E.head(3).dot(E90.head(3));
		double cosa=E.head(3).dot(E0.head(3));
		return std::atan2(sina,cosa);
	}


} // namespace Geometry
 
#endif // __projective_geometry_hxx
