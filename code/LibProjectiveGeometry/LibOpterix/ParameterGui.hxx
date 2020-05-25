// Created by Andre Aichert on Mon Feb 19th 2018
// Use of this file requires LibGetSet (https://github.com/aaichert/LibGetSet)
#ifndef __parameter_gui_hxx
#define __parameter_gui_hxx

#include <GetSet/GetSetObjects.h>
// #include "ParameterModel.hxx"

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

		ParameterGUI(const std::vector<std::string>& _parameter_names, const GetSetGui::Section& _section, bool make_collapsible=false)
			: parameter_names(index_prefix(_parameter_names))
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

		// Some convenient (conventional) getters.
		std::vector<bool  > getActive     () const { return gui_getValues<bool  >("Active"     ); }
		std::vector<double> getLowerBounds() const { return gui_getValues<double>("Lower Bound"); }
		std::vector<double> getUpperBounds() const { return gui_getValues<double>("Upper Bound"); }
		std::vector<double> getStep       () const { return gui_getValues<double>("Step"       ); }
		std::vector<double> getValues     () const { return gui_getValues<double>("Value"      ); }

		// Some convenient (conventional) setters.
		ParameterGUI& setActive     (const std::vector<bool  >& v) { return gui_setValues<bool  >("Active"     , v); }
		ParameterGUI& setLowerBounds(const std::vector<double>& v) { return gui_setValues<double>("Lower Bound", v); }
		ParameterGUI& setUpperBounds(const std::vector<double>& v) { return gui_setValues<double>("Upper Bound", v); }
		ParameterGUI& setStep       (const std::vector<double>& v) { return gui_setValues<double>("Step"       , v); }
		ParameterGUI& setValues     (const std::vector<double>& v) { return gui_setValues<double>("Value"      , v); }

		// And some declaration funcrions.
		ParameterGUI& declareActive() { return gui_setValues<bool  >("Active"      ); }
		ParameterGUI& declareBounds() {        gui_setValues<double>("Lower Bound");	
		                                return gui_setValues<double>("Upper Bound"); }
		ParameterGUI& declareStep()   { return gui_setValues<double>("Step"       ); }
		ParameterGUI& declareValues() { return gui_setValues<double>("Value"      ); }

	};
	
} // namespace LibOpterix


#endif // __parameter_gui_hxx
