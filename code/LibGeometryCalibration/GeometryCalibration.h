#ifndef __geometry_calibration_h
#define __geometry_calibration_h

#include <GetSet/GetSet.hxx>
#include <GetSet/GetSetObjects.h>

#include <NRRD/nrrd_image.hxx>

#include <LibProjectiveGeometry/ProjectiveGeometry.hxx>
#include <LibProjectiveGeometry/EigenToStr.hxx>

namespace VolumeRendering {
	class VoxelData;
	class Raycaster;
} // namespace VolumeRendering 

namespace Calibration {

	/// An interface for generating bead phantoms
	class BeadPhantomDescriptor : public GetSetGui::Configurable {
	public:
		/// Get location and radius for all beads.
		virtual std::vector<Eigen::Vector4d> getBeads() const = 0;
	};
	
	/// An interface for generating candidate matches to a set of 3D Points
	class BeadPhantomDetection : public GetSetGui::Configurable {
	public:
		std::vector<Eigen::Vector4d> beads;  //< 3D Phantom Beads Location and Radius

		/// Compute candidate matching. Returns map from 2D bead index to 3D bead index.
		virtual std::map<int,int> match(const std::vector<Eigen::Vector3d>& detected_beads) const = 0;

		/// Estimate good threshold to separate large and small beads
		static double computeRadiusThresholdMM(const std::vector<Eigen::Vector4d>& phantom_beads);

		/// Estimate good threshold to separate large and small beads
		static double computeRadiusThresholdPx(const std::vector<Eigen::Vector3d>& detected_beads);

		/// Declare types and default values for all properties.
		void gui_declare_section (const GetSetGui::Section& section);

		/// Retreive current values from GUI
		void gui_retreive_section(const GetSetGui::Section& section);

	};

	/// Class which uses a bead-phantom to analyze projection images for geometric calibration.
	class GeometryCalibration {
	public:
		/// Perform calibration based on matche candidates between 2d and 3d beads.
		Geometry::ProjectionMatrix calibrate(const std::map<int,int>& candidate_matched_2d_to_3d, const std::vector<Eigen::Vector3d>& detected_beads);
	};

	/// A class to simulate a BeadPhantom
	class BeadPhantomSimulator : public GetSetGui::Configurable {
	public:

		BeadPhantomSimulator();
		BeadPhantomSimulator(const BeadPhantomSimulator& other);
		~BeadPhantomSimulator();

		/// Not part of the object state. Merely temporaries for fast multiple drr projections.
		struct DRR {
			VolumeRendering::VoxelData*     voxel_data    = 0x0;
			VolumeRendering::Raycaster*     raycaster     = 0x0;
		} mutable drr;

		/// Parameters for simulating a DRR of the phantom.
		struct Voxelization {
			Eigen::Vector3i                 voxel_number  = Eigen::Vector3i::Constant(512);
			Eigen::Vector3d                 voxel_spacing = Eigen::Vector3d::Constant(.4);
		} voxelization;

		/// Get model matrix. If model_vx2mm is zero, the default is a bounding box with the origin in the center.
		Geometry::RP3Homography	getModel() const;

		/// Voxelize beads into 3D voxel data. beads are in format X,Y,Z,radius
		void voxelize(NRRD::Image<float>& phantom, const std::vector<Eigen::Vector4d>& beads, GetSetGui::ProgressInterface &app) const;

		/// load existing 3D volume
		void load_volume(const NRRD::ImageView<float>& phantom);

		/// Make a projection. out will be re-allocated to correct image size. Output is single-channel. Must call voxelize or load_volume first.
		void project(NRRD::ImageView<float>& out, const Geometry::ProjectionMatrix& P) const;

		/// Declare types and default values for all properties.
		virtual void gui_declare_section (const GetSetGui::Section& section);
		/// Retreive current values from GUI
		virtual void gui_retreive_section(const GetSetGui::Section& section);

	};

} // namespace Calibration

#endif // __geometry_calibration_h
