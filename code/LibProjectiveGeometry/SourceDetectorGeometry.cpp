
#include "SourceDetectorGeometry.h"

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
		U=infinitePoint(m2.cross(m3).normalized())*pixel_spacing;
		V=infinitePoint(m1.cross(m3).normalized())*pixel_spacing;
		// The principal plane contains the center of projection and is orthogonal to the axes
		principal_plane=P_star.block<1,4>(2,0);
		// Focal length in millimeters, assuming it is identical in U and V
		Eigen::Vector3d V_dir=V.head(3)/ pixel_spacing;
		double f=m1.dot(m3.cross(V_dir));
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
	SourceDetectorGeometry::SourceDetectorGeometry(
		const RP3Point& center_of_projection, const RP3Point& detector_origin,
		const Eigen::Vector3d& u_axis_vector, const Eigen::Vector3d& v_axis_vector)
		: C(center_of_projection)
		, O(detector_origin)
		, U(infinitePoint(u_axis_vector))
		, V(infinitePoint(v_axis_vector))
	{}

	/// Figure out projection by detector coordinate system and source position. Axis vectors should be scaled to the size of one pixel.
	ProjectionMatrix SourceDetectorGeometry::makeProjectionMatrix(const Geometry::RP3Point& source_pos, const Geometry::RP3Point& detector_origin,
		const Eigen::Vector3d& u_axis_vector, const Eigen::Vector3d& v_axis_vector)
	{
		// Figure out detector plane.
		RP3Plane detector_plane;
		// Normal is gven by axis directions
		detector_plane.head(3)=(u_axis_vector.cross(v_axis_vector)).normalized();
		// Distance by detector origin
		detector_plane(3)=0;
		detector_plane(3)=-detector_plane.dot(detector_origin);
		// Orthogonal projection to plane taking into consideration pixel size
		Geometry::ProjectionMatrix P_E=Geometry::ProjectionMatrix::Zero();
		P_E.block<1,3>(0,0)=u_axis_vector.transpose();
		P_E.block<1,3>(1,0)=v_axis_vector.transpose();
		P_E(2,3)=u_axis_vector.norm()*v_axis_vector.norm();
		P_E=P_E*centralProjectionToPlane(source_pos,detector_plane);
		normalizeProjectionMatrix(P_E);
		return P_E;
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
		ProjectionMatrix P=makeProjectionMatrix(C,O,U.head(3),V.head(3));
		normalizeProjectionMatrix(P);
		return P;
	}

} // namespace Geometry
