// Created by Andre Aichert on Mon Feb 19th 2018
#ifndef __parameter_gui_hxx
#define __parameter_gui_hxx

#include "ParameterModel.hxx"

namespace LibOpterix {

	/// Prefix a vector of strings with a two-digit index
	std::vector<std::string> index_prefix(const std::vector<std::string>& _parameter_names)
	{
		std::vector<std::string> parameter_names(_parameter_names.size());
			for (int i = 0; i < (int)_parameter_names.size(); i++)
				parameter_names[i]=toString(i, 2) + " " + _parameter_names[i];
		return parameter_names;
	}

	/// Define a set of parameters for optimization with NLOpt
	class ParameterGUI {
		const std::vector<std::string> parameter_names;
		const GetSetGui::Section       section;
	public:

		ParameterGUI(const std::vector<std::string>& _parameter_names, const GetSetGui::Section& _section, bool make_collapsible=false, bool prefix=true)
			: parameter_names(prefix?index_prefix(_parameter_names):_parameter_names)
			, section(_section)
		{
			if (make_collapsible)
				for (int i = 0; i < (int)parameter_names.size(); i++)
					GetSetGui::Section(parameter_names[i],section).setCollapsed();
			else
				for (int i = 0; i < (int)parameter_names.size(); i++)
					GetSetGui::Section(parameter_names[i],section).setGrouped();
		}


		/// Read all fields with a certain property from GUI
		template <typename T>
		std::vector<T> gui_getValues(const std::string& prop="Value") const
		{
			std::string slash_prop=std::string("/")+prop;
			std::vector<T> values(parameter_names.size());
			for (int i = 0; i < (int)parameter_names.size(); i++)
				values[i]=GetSet<T>(parameter_names[i]+slash_prop,section);
			return values;
		}

		///// Write all fields with a certain property in GUI. Also used to declate key type.
		template <typename T>
		ParameterGUI& gui_setValues(const std::string& prop="Value", const std::vector<T>& values=std::vector<T>())
		{
			std::string slash_prop=std::string("/")+prop;
			for (int i = 0; i < (int)parameter_names.size(); i++) {
				GetSet<T> key(parameter_names[i]+slash_prop,section);
				if (values.size()>i) key=values[i];
			}
			return *this;
		}

		/// Get set of indices of only active parameters.
		std::set<int> getActiveSet() const
		{
			auto active=getActive();
			std::set<int> active_set;
			for (int i=0;i<(int)active.size();i++)
				if (active[i]) active_set.insert(i);
			return active_set;
		}

		// Special function to set only active parameter values.
		ParameterGUI& setActiveValues(const std::vector<double>& v) {
			auto active=getActive();
			auto x     =getValues();
			for (int i=0,j=0;i<(int)active.size();i++)
				if (active[i]) x[i]=v[j++];
			return setValues(x);
		}

		// Some convenient (conventional) getters.
		std::vector<bool  > getActive     () const { return gui_getValues<bool  >("Active"     ); }
		std::vector<double> getLowerBounds() const { return gui_getValues<double>("Lower Bound"); }
		std::vector<double> getUpperBounds() const { return gui_getValues<double>("Upper Bound"); }
		std::vector<double> getStep       () const { return gui_getValues<double>("Step"       ); }
		std::vector<double> getValues     () const { return gui_getValues<double>("Value"      ); }

		// Some convenient (conventional) setters.
		ParameterGUI& setActive      (const std::vector<bool  >& v) { return gui_setValues<bool  >("Active"     , v); }
		ParameterGUI& setLowerBounds (const std::vector<double>& v) { return gui_setValues<double>("Lower Bound", v); }
		ParameterGUI& setUpperBounds (const std::vector<double>& v) { return gui_setValues<double>("Upper Bound", v); }
		ParameterGUI& setStep        (const std::vector<double>& v) { return gui_setValues<double>("Step"       , v); }
		ParameterGUI& setValues      (const std::vector<double>& v) { return gui_setValues<double>("Value"      , v); }


		// And some declaration funcrions.
		ParameterGUI& declareActive() { return gui_setValues<bool  >("Active"     ); }
		ParameterGUI& declareBounds() {        gui_setValues<double>("Lower Bound");	
		                                return gui_setValues<double>("Upper Bound"); }
		ParameterGUI& declareStep()   { return gui_setValues<double>("Step"       ); }
		ParameterGUI& declareValues() { return gui_setValues<double>("Value"      ); }


		// Finally some utility functions (since more often than not we need restricted vectors)
		std::vector<double> getActiveLowerBounds() const { return LibOpterix::restrict_vector(getLowerBounds(), getActive() ); }
		std::vector<double> getActiveUpperBounds() const { return LibOpterix::restrict_vector(getUpperBounds(), getActive() ); }
		std::vector<double> getActiveStep       () const { return LibOpterix::restrict_vector(getStep       (), getActive() ); }
		std::vector<double> getActiveValues     () const { return LibOpterix::restrict_vector(getValues     (), getActive() ); }
		

		/// Randomize parameter values
		ParameterGUI& randomize(double relative_range=0.5, bool active_only=true) {
			auto active=getActive();
			auto lb    =getLowerBounds();
			auto ub    =getUpperBounds();
			auto x     =getValues();
			for (int i=0;i<(int)active.size();i++)
				if (active[i] || !active_only)
				{
					double r=(double)rand()/RAND_MAX;
					x[i]+=relative_range*((1.0-r)*lb[i]+r*ub[i]);
				}
			return setValues(x);
		}

		/// Set all values to zero. Careful: ignored any default values possibly in your model.
		ParameterGUI& reset(bool active_only=true) {
			auto active=getActive();
			auto x=getValues();
			for (int i=0;i<(int)x.size();i++) if (active[i] || !active_only) x[i]=0;
			return setValues(x);
		}

	};
	
} // namespace LibOpterix


#endif // __parameter_gui_hxx
