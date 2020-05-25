#ifndef __phantompds2_hxx
#define __phantompds2_hxx

#include <GeometryCalibration.h>

#include <NRRD/nrrd_image.hxx>


namespace Calibration {

	/// A description of a PDS2 bead phantom
	struct PhantomDescriptorPDS2 : public BeadPhantomDescriptor
	{	
		/// A sequence of small (false) and big (true) beads, with all unambiguous sub-sequences of 8.
		static const std::vector<bool>& defaultBeadSequence();

		/// The helix of beads
		struct Helix {
			double height           = 206;
			double offset           = 33;
			double radius           = 67;
			double angularIncrement = 9.0 / 180.0 * Geometry::Pi;
		} helix;
		/// The plastic cylinder
		struct Cylinder {
			double outerRadius      = 72;
			double innerRadius      = 62;
			double density          = 0.125;
		} cylinder;
		/// The actual beads
		struct Beads {
			double axialIncrement   = 1.308411215;
			double smallDiameter    = 1.6;
			double largeDiameter    = 3.2;
			double density          = 1.0;
		} beads;
		
		/// Sequence of small and big beads
		std::vector<bool> largeBeadSequence = defaultBeadSequence();
		
		/// Create a bead phantom from a PDS2 descriptor
		virtual std::vector<Eigen::Vector4d> getBeads() const;

		/// Adds a plastic cylinder to an existing voxelized phantom. 2do
		// void addPlasticCylinder(NRRD::ImageView<float> beadphantom, const Geometry::RP3Homography& model_transform);
	
		/// Find a given code within the largeBeadSequence.
		static int computeBeadNumber(const std::vector<bool>& code, const std::vector<bool>& largeBeadSequence);

		/// Declare types and default values for all properties.
		virtual void gui_declare_section (const GetSetGui::Section& section);
		/// Retreive current values from GUI
		virtual void gui_retreive_section(const GetSetGui::Section& section);

	};
	
	/// A class to find 2D/3D matches in projections of the PDS2 phantom
	class PhantomDetectionPDS2 : public BeadPhantomDetection
	{
	public:
		struct ParamMatching {
			double distance_min         = 10; //< Min. distance of closest neighboring bead
			double angular_tolerance    = 10; //< Neighborhood: Max. degrees deviation from line
		} paramMatching;

		/// Compute candidate matching. Returns map from 2D bead index to 3D bead index.
		virtual std::map<int,int> match(const std::vector<Eigen::Vector3d>& detected_beads) const;
		
		/// Declare types and default values for all properties.
		void gui_declare_section (const GetSetGui::Section& section);
		/// Retreive current values from GUI
		void gui_retreive_section(const GetSetGui::Section& section);

		mutable std::vector<Eigen::Vector2d> debug_dirs; //< Local estimates direction of helix.  DELETE  just for debugging.

	};

} // namespace Calibration

#endif // __phantompds2_hxx
