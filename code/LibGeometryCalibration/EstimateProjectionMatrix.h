#ifndef __estimate_projection_matrix_h
#define __estimate_projection_matrix_h

#include <LibProjectiveGeometry/ProjectiveGeometry.hxx>
#include <LibProjectiveGeometry/EigenToStr.hxx>

#include <GetSet/GetSet.hxx>
#include <GetSet/GetSetObjects.h>

#include <NRRD/nrrd_image.hxx>

namespace Calibration {

	/// An abstract class which supports generating candidate matches to a set of 3D Points
	class EstimateProjectionMatrix : public GetSetGui::Configurable {
	public:
		struct ICP {
			int    num_ietartions            =  2;    //< The maximum number of iterations allowed in the algorithm.
			double inliner_tolerance_px      = 10;    //< A distance threshold for closest bead for the match to be considered correct.
		} icp;
		struct RANSAC {
			double inliner_tolerance_px      = 4;     //< When the reprojection error of a point is larger, we consider it an outlier.
			int    max_ietartions            = 100;   //< The maximum number of iterations allowed in the algorithm.
			double min_inliner_rel           = 0.5;   //< Stopping criteria: minimum proportion of inliers.
		} ransac;
		struct Algorithms {
			bool   use_ransac                = true;  //< Use Random Sample Consensus
			bool   use_icp                   = false; //< Use Iterative Closest Point for more matches
			bool   use_non_linear_refinement = false; //< 2do not implemented
		} algorithms;

		mutable std::map<int,int> inlier_set; // debug

		/// Compute candidate matching. Returns map from 2D bead index to 3D bead index.
		Geometry::ProjectionMatrix estimateProjection(
				const std::map<int,int>& matching,
				const std::vector<Eigen::Vector3d>& detected_beads,
				const std::vector<Eigen::Vector4d>& phantom_beads
			) const;
	
		/// Declare types and default values for all properties.
		virtual void gui_declare_section (const GetSetGui::Section& section);
		/// Retreive current values from GUI
		virtual void gui_retreive_section(const GetSetGui::Section& section);

	};


} // namespace Calibration

#endif // __estimate_projection_matrix_h
