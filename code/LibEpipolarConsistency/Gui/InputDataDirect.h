// Created by A. Aichert on Tue Aug 8th 2017.
#ifndef __input_data_direct
#define __input_data_direct

#include <GetSet/GetSetObjects.h>
#include <GetSet/ProgressInterface.hxx>

#include "PreProccess.h"
#include "ComputeRadonIntermediate.hxx"

namespace EpipolarConsistency {

	/// Loading and preparation of (mostly) FDCT data for Epipolar Consistency.
	struct InputDataDirect : public GetSetGui::Configurable {

		InputDataDirect();
		virtual ~InputDataDirect();

		/// Pre-processing of X-ray projection images. Optional: can also load pre-computed DTRs.
		struct Projections {
			std::string               projection_matrices;					//< Geometry of the scan for motion compenstation. (overrides image's meta info)
			std::vector<std::string>  image_files;							//< One 3D stack or several X-ray projection images. 2do other file formats via ITK
			PreProccess               pre_process;							//< Properties related to pre-processing the projection data.
		} projections;

		/// Storing and visualizing output results
		struct AdcancedImage {
			double                    mm_per_px=0.308;						//< Pixel Spacing
			int                       skip_projections=0;					//< For large data sets, one may reduce the memory by skipping some projections.
			bool                      show_pre_processed_imgs=true;			//< Displays the projection image after pre-processing
			std::string               basename_pre_processed_images;		//< If specified, pre-processed images are stored here.
			std::vector<std::string>  pre_processed_images_files;			//< Loaded if non-empty and set if basename_pre_processed_images is non-empty.
		} advanced;

		/// Declare default values.
		void gui_declare_section (const GetSetGui::Section& section);

		/// Retreive current values from GUI.
		void gui_retreive_section(const GetSetGui::Section& section);

		/// Retreive information about input data from GUI and load data. Does nothing if data is already valid unless force_reload is set.
		bool loadData(std::vector<Geometry::ProjectionMatrix>& Ps, std::vector<UtilsCuda::BindlessTexture2D<float>*>& Is, GetSetGui::ProgressInterface& progress, bool force_reload=false);
		
		/// Update image sources from GUI and prepare for loading multipe images via loadPreprocessedImage(...)
		int prepareForLoadingPreprocessedImage() const;

		/// Load a single pre-processed image. Please call prepareForLoadingPreprocessedImage() before use. 
		virtual bool loadPreprocessedImage(NRRD::Image<float>& img, Geometry::ProjectionMatrix& P, int i);

		/// Load projection matrices
		static std::vector<Geometry::ProjectionMatrix> loadProjectionMatrices(const std::string& projection_matrices);

		/// Convenient access to projections.mm_per_px (because it is commonly needed for visualization)
		virtual double getPixelSpacing() const;

		/// Handles GUI changes automatically. (currently does nothing)
		void gui_notify(const std::string& section, const GetSetInternal::Node&) {}

		/// Temporary information for image loading.
		class ImageSource {
			NRRD::ImageStack<float>                *stack; //< Loading slices of a 3D stack (or null for multiple 2D images)
			std::vector<std::string>                files; //< Paths to image file(s)
			std::vector<Geometry::ProjectionMatrix> Ps_stack;
			ImageSource(const ImageSource&);

		public:
			ImageSource(const std::vector<std::string>& _files);
			~ImageSource();

			/// Number of available images. Invalid if less than two.
			int numImages() const;

			/// Load an image from disk
			bool getImage(NRRD::Image<float>& I, Geometry::ProjectionMatrix& P, int i) const;

		} mutable *image_source; //< Temporary, not part of class state.

	};

	/// An advanced GetSet Object based on the simple Configurable 
	class InputDataDirectGui : public GetSetGui::Object, public InputDataDirect {
	protected:
		std::vector<Geometry::ProjectionMatrix>             Ps;
		std::vector<UtilsCuda::BindlessTexture2D<float>*>	Is;

		/// Handle (Re-)Load button and visualization stuff.
		virtual void gui_notify(const std::string& section, const GetSetInternal::Node&);
		
		/// Adds a re-load button to the usual InputDataDirect Gui.
		virtual void gui_declare_section(const GetSetGui::Section& section);

	public:
		InputDataDirectGui(const GetSetGui::Section& section, GetSetGui::ProgressInterface* app=0x0);

		/// Attempt to load data. If force_reload is not set and Ps and rdas are already set, just returns true.
		virtual bool loadData(bool force_reload=false);

		/// Projection matrices. Make sure to first call laodData(bool force_reload) and check for success.
		const std::vector<Geometry::ProjectionMatrix>&			getProjectionMatrices() const;

		/// Projection Images. Make sure to first call laodData(bool force_reload) and check for success.
		const std::vector<UtilsCuda::BindlessTexture2D<float>*>& getProjectionImages() const;

	};

} // namespace EpipolarConsistency

#endif // __input_data_direct
