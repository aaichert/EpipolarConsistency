#ifndef _beaddetection_h
#define _beaddetection_h

#include <LibProjectiveGeometry/ProjectiveGeometry.hxx>
#include <NRRD/nrrd_image.hxx>

#include <GetSet/GetSetObjects.h>

#include <LibUtilsQt/GraphicsItems.hxx>
#include <LibUtilsQt/GraphicsItemConvexMesh.hxx>

namespace Calibration {

	/// Detect bright round blobs in an image.
	struct BeadDetection : public GetSetGui::Configurable {
			/// Parameers for the fast radial symmetry transform
			struct ParamsFRST {
				std::vector<double> beadRadiiPx=stringToVector<double>("4;8");
				double gaussian_sigma    = 3.68;
				int    gaussian_width    = 5;
				double threshold_frst    = 0.1;
				double threshold_gradient= 0;
			} paramsFRST;

			/// Result: Detected beads
			std::vector<Geometry::RP2Point> beads;
			/// Intermediate result: FRST (after thresholding)
			NRRD::Image<float> frst;
			/// Intermediate result: Connected components of thresholded FRST.
			NRRD::Image<unsigned short> components;

			/// Visualize results
			static void overlayDetectedBeads(
				const std::vector<Geometry::RP2Point>& result,
				GraphicsItems::Group& overlay,
				double last_component_threshold=0);

			/// Analyze single channel 2D image for bright beads. Returns (X,Y,radius).
			const std::vector<Eigen::Vector3d>& process(const NRRD::ImageView<float>& image);

			virtual void gui_declare_section (const GetSetGui::Section &section);
			virtual void gui_retreive_section(const GetSetGui::Section &section);
	
	};

#undef small

	/// Differentiate between large and small bright blobs
	struct BeadDetectionTwoSize : public GetSetGui::Configurable {
		
		double min_distance_px=10; //< If two detections are less than min_distance_px apart, they are handled according to prefer_beads
		BeadDetection large;
		BeadDetection small;
		/// Result: Detected beads
		std::vector<Geometry::RP2Point> beads;

		/// Analyze single channel 2D image for bright beads. Returns (X,Y,radius).
		const std::vector<Eigen::Vector3d>& process(const NRRD::ImageView<float>& image);
		
		virtual void gui_declare_section (const GetSetGui::Section &section);
		virtual void gui_retreive_section(const GetSetGui::Section &section);
	};

	/// BeadDetectionTwoSize with testing functions
	struct BeadDetectionGui : public BeadDetectionTwoSize, public GetSetGui::Object {
		BeadDetectionGui(const GetSetGui::Section& section, GetSetGui::ProgressInterface *app);
		virtual void gui_init();
		virtual void gui_notify(const std::string& relative_section, const GetSetInternal::Node& node);
	};

} // Calibration

#endif // _beaddetection_h
