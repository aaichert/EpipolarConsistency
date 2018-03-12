// Created by A. Aichert on Tue Aug 8th 2017.
#ifndef __input_data_radon_intermediate
#define __input_data_radon_intermediate

#include <GetSet/GetSetObjects.h>
#include <GetSet/ProgressInterface.hxx>

#include "InputDataDirect.h"
#include "ComputeRadonIntermediate.hxx"

namespace EpipolarConsistency {

	/// Loading and preparation of (mostly) FDCT data for Epipolar Consistency.
	struct InputDataRadonIntermediate : public InputDataDirect {

		RadonIntermediateFunction     radon_intermediate;					//< Computing the Radon intermediate functions

		/// Storing and visualizing output results (to recover more quickly when validating/evaluating)
		struct Advanced_DTR {
			bool                      show_radon_intermediate=true;			//< Displays the Radon Intermediate Function after computation
			std::string               basename_radon_intermediate;			//< If specified, Radon Intermediate Images are stored here.
			std::vector<std::string>  precomputed_radon_intermediate_files;	//< Loaded if non-empty and set if basename_radon_intermediate is non-empty.
		} advanced_dtr;

		/// Declare default values.
		void gui_declare_section (const GetSetGui::Section& section);

		// Retreive current values from GUI
		void gui_retreive_section(const GetSetGui::Section& section);

		/// Retreive information about input data from GUI and load data. Does nothing if data is already valid unless force_reload is set.
		bool loadData(std::vector<Geometry::ProjectionMatrix>& Ps, std::vector<EpipolarConsistency::RadonIntermediate*>& dtrs, GetSetGui::ProgressInterface& progress, bool force_reload=false);
	
		/// Handles GUI changes automatically. (currently does nothing)
		void gui_notify(const std::string& section, const GetSetInternal::Node&) {}

	};

	/// An advanced GetSet Object based on the simple Configurable 
	class InputDataRadonIntermediateGui : public GetSetGui::Object, public InputDataRadonIntermediate {
	protected:
		std::vector<Geometry::ProjectionMatrix>              Ps;
		std::vector<EpipolarConsistency::RadonIntermediate*> dtrs;

		/// Handle (Re-)Load button and visualization stuff.
		virtual void gui_notify(const std::string& section, const GetSetInternal::Node&);
		
		/// Adds a re-load button to the usual InputDataRadonIntermediate Gui.
		virtual void gui_declare_section(const GetSetGui::Section& section);

	public:
		InputDataRadonIntermediateGui(const GetSetGui::Section& section, GetSetGui::ProgressInterface* app=0x0);

		/// Attempt to load data. If force_reload is not set and Ps and rdas are already set, just returns true.
		bool loadData(bool force_reload);

		/// Projection matrices. Make sure to first call laodData(bool force_reload) and check for success.
		const std::vector<Geometry::ProjectionMatrix>&              getProjectionMatrices() const;

		/// Radon Intermediate Functions. Make sure to first call laodData(bool force_reload) and check for success.
		const std::vector<EpipolarConsistency::RadonIntermediate*>& getRadonIntermediateFunctions() const;

	};

} // namespace EpipolarConsistency

#endif // __input_data_radon_intermediate
