#ifndef __source_detector_geometry_h
#define __source_detector_geometry_h
// Created by A. Aichert on Mon Jan 15th 2018
// A useful class to relate 3D X-ray detector coordinates with pixels of a projection image.

#include "ProjectionMatrix.h"

namespace Geometry
{
	/// A class to support computations in world units directly on the physical detector.
	struct SourceDetectorGeometry {
		// Vectors defining a projection matrix
		RP3Point C;  //< Center of projection (i.e. source position)
		RP3Point O;  //< Detector origin in three-space
		RP3Point U;  //< Inifinte point in u-direction
		RP3Point V;  //< Inifinte point in v-direction
		// Additional related objects for utility
		RP3Point principal_point_3d;      //< Principal Point in three-space
		RP3Plane principal_plane;         //< Principal plane
		RP3Plane image_plane;             //< Image plane
		RP3Homography central_projection; //< Projection to image plane

		/// Figure out projection by detector coordinate system and source position. Axis vectors should be scaled to the size of one pixel.
		static ProjectionMatrix makeProjectionMatrix(
			const Geometry::RP3Point& center_of_projection, const Geometry::RP3Point& detector_origin,
			const Eigen::Vector3d& u_axis_vector, const Eigen::Vector3d& v_axis_vector);

		/// Figure out source-detector geometry by decomposing a projection matrix
		SourceDetectorGeometry(const ProjectionMatrix&P, double pixel_spacing);

		/// Source-detector given by physical measurements. u/v_axis_vector should be scaled to size of one pixel in millimeters.
		SourceDetectorGeometry(
			const RP3Point& center_of_projection, const RP3Point& detector_origin,
			const Eigen::Vector3d& u_axis_vector, const Eigen::Vector3d& v_axis_vector);

		// Get the 3D coordinates of the pixel (u,v) in world units
		RP3Point point_on_detector(double u, double v) const;
		// Get the 3D coordinates of the image point x in world units
		RP3Point point_on_detector(const RP2Point& x) const;
		// Get the 3D coordinates of the image point x in world units
		RP3Point project_to_detector(const RP3Point& X) const;

		// Project 3D point to image pixel
		RP2Point project(RP3Point X) const;
		// Compute a projection matrix which relates 3D points to image pixels. See also: project(...)
		ProjectionMatrix projectionMatrix() const;

	};
	
} // namespace Geometry

#endif // __source_detector_geometry_h
