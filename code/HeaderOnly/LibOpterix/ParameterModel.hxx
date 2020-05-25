#ifndef __model_hxx
#define __model_hxx

#include <set>
#include <map>
#include <vector>
#include <string>
#include <stdexcept>

namespace LibOpterix {

	typedef std::map<std::string, std::set<int> > ParameterSets;

	/// Restrict values from expanded vector to only those elements for which the active vector at the same index is true.
	template <typename T>
	inline std::vector<T> restrict_vector(const std::vector<T>& expanded, const std::vector<bool>& active)
	{
		std::vector<T> restricted;
		for (int i=0;i<(int)active.size();i++)
			if (active[i]) restricted.push_back(expanded[i]);
		return restricted;
	}

	/// Return restricted vector with only the elements of expanded whose index is provided in active_set
	template <typename T>
	inline std::vector<T> restrict_vector(const std::vector<T>& expanded, const std::set<int>& active_set)
	{
		std::vector<T> restricted;
		restricted.reserve(active_set.size());
		for (auto it=active_set.begin();it!=active_set.end();++it)
			restricted.push_back(expanded[*it]);
		return restricted;
	}

	/// Return expanded vector by setting only those values  for which the active vector at the same index is true.
	template <typename T>
	std::vector<T>& expand_vector(const std::vector<T>& restricted, std::vector<T>& expanded, const std::vector<bool>& active)
	{
		int j=0;
		for (int i=0;i<(int)active.size();i++)
			if (active[i]) expanded[i]=restricted[j++];
		return expanded;
	}

	/// Return expanded vector by setting only those values whose index is provided in active_set
	template <typename T>
	std::vector<T>& expand_vector(const std::vector<T>& restricted, std::vector<T>& expanded, const std::set<int>& active_set)
	{
		int i=0;
		for (auto it=active_set.begin();it!=active_set.end();++it)
			expanded[*it]=restricted[i++];
		return expanded;
	}
	/// A full (or over-) parametrization of Object, of which only active parameters shall be optimized.
	struct AbstractParameterModel {
		/// Number of parameters in expanded parametrization.
		virtual int numberOfParameters() const =0;
		/// Number of parameters in restricted parametrization.
		virtual int numberOfParametersActive() const =0;
		/// Set of indices of active parameters.
		virtual const std::set<int>& activeParameters() const =0;
		/// Returns human readable short names of parameter with index 0<=i<numberOfParameters()
		virtual const std::vector<std::string>& parameterNames() const =0;
		/// Returns human readable short names of parameter with index 0<=i<numberOfParameters()
		virtual std::vector<std::string> parameterNamesActive() const {
			return restrict_vector(parameterNames(), activeParameters());
		}

		/// A vector of values only of active parameter.
		virtual std::vector<double> restrict() const =0;
		/// Get full parametrization. Optionally set active parameters. 
		virtual std::vector<double>& expand(const double *x_active=0x0) =0;
		/// Take current values from a vector of only active parameter values (see also: restrict()) and return updated current_values.
		virtual std::vector<double>& expand(const std::vector<double>& x_active)
		{
			if ((int)x_active.size()!=numberOfParametersActive())
				throw std::out_of_range("ParameterModel<Object>::expand(...) - Length of parameter vector does not match number of active parameters.");
			return expand(&x_active[0]);
		}
	};
	
	/// A Model parametrizing a specific Object
	template <typename Object>
	struct ParameterModel : public AbstractParameterModel { // FIXME protected

		// Derived classes are expected to define:
		// static const std::vector<std::string>& ParameterNames()
		// static const Parametrization::ParameterSets& ParameterSets()

		/// Human readable short names of parameters
		const std::vector<std::string>&	parameter_names;
		/// Values of parameters. Size of current_values is the same as parameter_names.
		std::vector<double>				current_values;
		/// Set of active parameters. Inidices address parameter_names/current_values.
		const std::set<int>				active_parameters;

	public:
		/// Define parameters and a set of active parameters. Throws if _active set includes invalid indices.
		ParameterModel(
			const std::vector<std::string>& _parameters,
			const std::set<int>& _active_set)
		: parameter_names(_parameters)
		, active_parameters(_active_set)
		, current_values(numberOfParameters(),0.0)
		{
			// Make sure all inidces in active set actually reference a parameter.
			int n=numberOfParameters();
			for (auto it=_active_set.begin();it!=_active_set.end();++it)
				if (*it<0||*it>=n)
					 throw std::invalid_argument("Parametrization::Model: Set of active parameters contains invalid indices." );
		}
		
		/// Returns human readable short names of parameter with index 0<=i<numberOfParameters()
		virtual const std::vector<std::string>& parameterNames() const {
			return parameter_names;
		}

		/// Total number of parameters. See also: numberOfParametersActive()
		virtual int numberOfParameters() const {
			return (int)parameter_names.size();
		}

		/// Number of active parameters (which are to be optimized)
		virtual int numberOfParametersActive() const {
			return (int)active_parameters.size();
		}

		/// Set of indices of active parameters.
		virtual const std::set<int>& activeParameters() const {
			return active_parameters;
		}

		/// Vector of only active parameters.
		virtual std::vector<double> restrict() const
		{
			std::vector<double> x_active(active_parameters.size());
			auto it=active_parameters.begin();
			for (int a=0; it!=active_parameters.end();++it,++a)
				x_active[a]=current_values[*it];
			return x_active;
		}

		/// Take current values from a vector of only active parameter values (see also: restrict()) and return updated current_values.
		virtual std::vector<double>& expand(const double *x_active=0x0)
		{
			if (x_active)
			{
				auto it=active_parameters.begin();
				for (int a=0; it!=active_parameters.end();++it,++a)
					current_values[*it]=x_active[a];
			}
			return current_values;
		}

		/// Set current values from (expanded) vector of parameters. See als: expand()[i] to get value.
		ParameterModel<Object>& setValues(const double *x)
		{
			for (int i=0;i<(int)current_values.size();i++)
				current_values[i]=x[i];
			return *this;
		}

		/// Set current value (indexed as expanded). See als: expand()[i] to get value.
		ParameterModel<Object>& setValue(int i, double x)
		{
			current_values[i]=x;
			return *this;
		}

		/// Return the object for the current parameter values.
		virtual Object getInstance() const = 0;
		
	};

} // Parametrization

#endif // __model_hxx
