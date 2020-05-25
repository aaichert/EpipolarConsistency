
#include "SourceDetectorGeometry.h"
#include "EstimateProjectionMatrix.h"


#include <iostream> // TODO remove



namespace Geometry
{
	/// Figure out source-detector geometry by decomposing a projection matrix
	SourceDetectorGeometry::SourceDetectorGeometry(const ProjectionMatrix&P, double pixel_spacing)
	{
		// Handle scale and orientation by assuming a right-handed image coordinate
		// system with m3 being a unit vector in direction of principal ray.
		ProjectionMatrix P_star=P;
		normalizeProjectionMatrix(P_star);
		// Get center or projection
		C=getCameraCenter(P_star);
		// Get axis plane normals
		Eigen::Vector3d m1=P_star.block<1,3>(0,0);
		Eigen::Vector3d m2=P_star.block<1,3>(1,0);
		// Get principal ray direction
		Eigen::Vector3d m3=P_star.block<1,3>(2,0);
		// Account for left-handed coordinate systems if desired
		// Get axis directions (unit vectors, scaled to pixels)
		U=infinitePoint(m3.cross(m2).normalized())*pixel_spacing;
		V=infinitePoint(m3.cross(m1).normalized())*pixel_spacing;
		// The principal plane contains the center of projection and is orthogonal to the axes
		principal_plane=P_star.block<1,4>(2,0);
		// Focal length in millimeters, assuming it is identical in U and V
		Eigen::Vector3d V_dir=V.head(3)/ pixel_spacing;
		double f=m1.dot(V_dir.cross(m3));
		// The image plane is parallel to the principal plane at a distance of f (focal length)
		image_plane=principal_plane;
		image_plane(3)-=f*pixel_spacing;
		// Hack for left-handed coordinate systems.
		if (pixel_spacing<0)  V*=-1;
		// Projection to image plane
		central_projection=centralProjectionToPlane(C,image_plane);
		// Orthogonal projection of center of projection to image plane.
		principal_point_3d=C-infinitePoint(m3)*image_plane.dot(C);
		// Principal point in pixels
		RP2Point pp=P_star*principal_point_3d;
		pp/=pp(2);
		// Location of origin ( principal_point_3d should map to principal in point 2D) 
		O=principal_point_3d-U*pp[0]-V*pp[1];
		return;
	}
	
	/// Source-detector given by physical measurements. u/v_axis_vector should be scaled to size of one pixel in millimeters.
	SourceDetectorGeometry::SourceDetectorGeometry(const Geometry::RP3Point& source_pos, const Geometry::RP3Point& detector_origin,
		const Eigen::Vector3d& u_axis_vector, const Eigen::Vector3d& v_axis_vector)
	{
		C=source_pos;
		O=detector_origin;	
		U.head(3)=u_axis_vector;
		V.head(3)=v_axis_vector;
		U[3]=V[3]=0;
		// Line at infinity for planes parallel to image
		Geometry::RP3Line L_normal=Geometry::join_pluecker(U.normalized(),V.normalized());
		// Plane normal
		Eigen::Vector3d normal=Geometry::pluecker_moment(L_normal);
		// Plane through detector origin: image plane!
		image_plane=Geometry::join_pluecker(L_normal,O);
		// Plane through source position: principal plane!
		principal_plane=Geometry::join_pluecker(L_normal,C);
		// Principal point: Orthogonal projection of source position to image plane
		// First step: compute line orthogonal to detector through C
		Geometry::RP3Line ortholine=Geometry::join_pluecker(Geometry::infinitePoint(normal),C);
		// Second step: intersect that line with the image plane
		principal_point_3d=Geometry::meet_pluecker(ortholine,image_plane);
		// Finally set up matrix that projects to detector
		central_projection=Geometry::centralProjectionToPlane(C,image_plane);
		// We have now fully defined the source-detector-geometry object.
	}

	/// Get the 3D coordinates of the pixel (u,v) in world units
	RP3Point SourceDetectorGeometry::point_on_detector(double u, double v) const {
		return point_on_detector(RP2Point(u,v,1));
	}

	/// Get the 3D coordinates of the image point x in world units
	RP3Point SourceDetectorGeometry::point_on_detector(const RP2Point& x) const {
		return (U*x[0]+V*x[1])/x[2]+O;
	}

	/// Get the 3D coordinates of the image point x in world units
	RP3Point SourceDetectorGeometry::project_to_detector(const RP3Point& X) const {
		return RP3Point(central_projection*X);
	}

	/// Project 3D point to image pixel
	RP2Point SourceDetectorGeometry::project(RP3Point X) const {
		RP3Point in_plane=central_projection*X;
		in_plane=in_plane/in_plane(3)-O;
		return RP2Point(U.dot(in_plane), V.dot(in_plane), U.norm()*V.norm());
	}


	/// Compute a projection matrix which relates 3D points to image pixels. See also: project(...)
	ProjectionMatrix SourceDetectorGeometry::projectionMatrix() const
	{
		// This pece of code is not a good solution.
		// P can be computed directly from sdg object.
		// No DLT needed.

		Geometry::RP3Point Xv[]={
			O  ,
			O+C,
			O  +U,
			O+C+U,
			O    +V,
			O+C  +V,
			O  +U+V,
			O+C+U+V
		};
		
		Geometry::RP2Point xv[]={
			Geometry::RP2Point(0,0,1),
			Geometry::RP2Point(0,0,1),
			Geometry::RP2Point(1,0,1),
			Geometry::RP2Point(1,0,1),
			Geometry::RP2Point(0,1,1),
			Geometry::RP2Point(0,1,1),
			Geometry::RP2Point(1,1,1),
			Geometry::RP2Point(1,1,1)
		};

		std::vector<Geometry::RP3Point> X(Xv,Xv+8);
		std::vector<Geometry::RP2Point> x(xv,xv+8);

		auto P= Geometry::dlt(x,X);

		std::cout << "P=\n" << P << std::endl;
		std::cout << "C=\n" << Geometry::getCameraCenter(P) << std::endl;

		SourceDetectorGeometry sdg(P,V.norm());

		std::cout << "O=\n" << sdg.O << std::endl;
		std::cout << "U=\n" << sdg.U << std::endl;
		std::cout << "V=\n" << sdg.V << std::endl;

		return P;
	}

} // namespace Geometry
