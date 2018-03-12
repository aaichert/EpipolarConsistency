// Created by A. Aichert on Mon Mar 27st 2017.
#ifndef __compute_radon_intermediate
#define __compute_radon_intermediate

#include <LibProjectiveGeometry/ProjectionMatrix.h>
#include <LibEpipolarConsistency/RadonIntermediate.h>

#include <GetSet/GetSetObjects.h>
#include <GetSet/ProgressInterface.hxx>

#include <LibUtilsQt/Figure.hxx>

#include <NRRD/nrrd_image_stack.hxx>

#include <Utils/Projtable.hxx>

#include "PreProccess.h"

#include <map>
#include <string>

namespace EpipolarConsistency
{
	//
	// Computing Radon Intermediate and Filter
	//

	struct RadonIntermediateFunction : public GetSetGui::Configurable {
		enum Filter {
			Derivative=0, Ramp=1, None=2,
		} filter=Derivative;				//< Filter Radon transform by derivative in distance direction.
		struct NumberOfBins {
			int angle=768;					//< Size of the Radon transform in angle-direction.
			int distance=768;				//< Size of the Radon transform in distance-direction.
		} number_of_bins;

		/// Declare default values.
		void gui_declare_section (const GetSetGui::Section& section)
		{
			// RadonIntermediate
			GetSet<int>("Number Of Bins/Angle"    ,section,number_of_bins.angle                     ).setDescription("");
			GetSet<int>("Number Of Bins/Distance" ,section,number_of_bins.distance                  ).setDescription("");
			GetSetGui::Enum("Distance Filter"     ,section,filter).setChoices("Derivative;Ramp;None").setDescription("");
			section.subsection("Number Of Bins").setGrouped();
		}
		
		// Retreive current values from GUI
		void gui_retreive_section(const GetSetGui::Section& section)
		{
			filter                 =(Filter)(GetSet<int>("Distance Filter"         ,section).getValue());
			number_of_bins.angle   =         GetSet<int>("Number Of Bins/Angle"    ,section);
			number_of_bins.distance=         GetSet<int>("Number Of Bins/Distance" ,section);
		}

		/// Load and process Radon Intemediate Functions. 
		RadonIntermediate* compute(NRRD::ImageView<float>& img, Geometry::ProjectionMatrix *P=0x0, double *mm_per_px=0x0)
		{
			// Compute Radon Intermediate
			int n_alpha=number_of_bins.angle;
			int n_t=number_of_bins.distance;
			auto * dtr=new EpipolarConsistency::RadonIntermediate( img, n_alpha,n_t, filter==0);

			// Store projection matrix in NRRD header, if provided
			if (P)         img.meta_info["Original Image/Projection Matrix"] = toString(*P);
			if (mm_per_px) img.meta_info["Original Image/Pixel Spacing"    ] = toString(*mm_per_px);

			return dtr;
		}

	};


} // namespace EpipolarConsistency

#endif // __compute_radon_intermediate
