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

/////////////////////////
// JUST FOR TESTING
#include <NRRD/nrrd_lowpass.hxx>
/////////////////////////

namespace EpipolarConsistency
{
	//
	// Computing Radon Intermediate and Filter
	//

	struct RadonIntermediateFunction : public GetSetGui::Configurable {

		/// Filter applied to Radon transform.
		RadonIntermediate::Filter filter=RadonIntermediate::Derivative;

		/// Function applied to each value in the Radon transform
		RadonIntermediate::PostProcess  post_process=RadonIntermediate::Identity;

		/// Size of the Radon transform.
		struct NumberOfBins {
			int angle=768;					//< Size of the Radon transform in angle-direction.
			int distance=768;				//< Size of the Radon transform in distance-direction.
		} number_of_bins;

		/// Declare default values.
		void gui_declare_section (const GetSetGui::Section& section)
		{
			// RadonIntermediate
			GetSet<int>       ("Number Of Bins/Angle"             , section, number_of_bins.angle      ).setDescription("Number of Radon bins in alpha-direction");
			GetSet<int>       ("Number Of Bins/Distance"          , section, number_of_bins.distance   ).setDescription("Number of Radon bins in t-direction");
			section.subsection("Number Of Bins").setGrouped();
			GetSetGui::Enum   ("Distance Filter"                  , section, filter                    ).setChoices("Derivative;Ramp;None")
				.setDescription("A homogeneous function of degree two must be applied for the consistency condition to hold. This becomes a Ramp or derivative filter in t-direction.");
			GetSetGui::Enum   ("Post Process"                     , section, post_process              ).setChoices("Identity;sgn(x)*sqrt(abs(x));sgn(x)*log(abs(x)-1)")
				.setDescription("A function applied to the Radon transform after computation. Function: x, sgn(x)*sqrt(abs(x)) or sgn(x)*log(abs(x)+1).");
		}
		
		// Retreive current values from GUI
		void gui_retreive_section(const GetSetGui::Section& section)
		{
			post_process              =(RadonIntermediate::PostProcess)(GetSet<int>("Post Process"                      , section).getValue());
			filter                    =(RadonIntermediate::Filter)     (GetSet<int>("Distance Filter"                   , section).getValue());
			number_of_bins.angle      =                                 GetSet<int>("Number Of Bins/Angle"              , section);
			number_of_bins.distance   =                                 GetSet<int>("Number Of Bins/Distance"           , section);
		}

		/// Load and process Radon Intemediate Functions. 
		RadonIntermediate* compute(NRRD::ImageView<float>& img, Geometry::ProjectionMatrix *P=0x0, double *mm_per_px=0x0)
		{
			// Compute Radon Intermediate
			int n_alpha=number_of_bins.angle;
			int n_t=number_of_bins.distance;
			auto * dtr=new EpipolarConsistency::RadonIntermediate( img, n_alpha,n_t, filter, post_process);


			// Store projection matrix in NRRD header, if provided
			if (P)         img.meta_info["Original Image/Projection Matrix"] = toString(*P);
			if (mm_per_px) img.meta_info["Original Image/Pixel Spacing"    ] = toString(*mm_per_px);

			return dtr;
		}

	};


} // namespace EpipolarConsistency

#endif // __compute_radon_intermediate
