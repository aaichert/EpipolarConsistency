#ifndef nlopt_utils_hxx
#define nlopt_utils_hxx

#include <nlopt.hpp>

#include <LibOpterix/Opterix.hxx>

#include <GetSet/GetSetObjects.h>

#include "ParameterGui.hxx"

namespace UtilsNLOpt {

	/// Produce a list of all NLopt algorithms by their identified enum value.
	inline const std::map<std::string,nlopt::algorithm>& nlopt_algorithms()
	{
		static std::map<std::string,nlopt::algorithm> algos;
		if (algos.empty())
		{
			algos["GN_DIRECT"]                 = nlopt::GN_DIRECT;
			algos["GN_DIRECT_L"]               = nlopt::GN_DIRECT_L;
			algos["GN_DIRECT_L_RAND"]          = nlopt::GN_DIRECT_L_RAND;
			algos["GN_DIRECT_NOSCAL"]          = nlopt::GN_DIRECT_NOSCAL;
			algos["GN_DIRECT_L_NOSCAL"]        = nlopt::GN_DIRECT_L_NOSCAL;
			algos["GN_DIRECT_L_RAND_NOSCAL"]   = nlopt::GN_DIRECT_L_RAND_NOSCAL;
			algos["GN_ORIG_DIRECT"]            = nlopt::GN_ORIG_DIRECT;
			algos["GN_ORIG_DIRECT_L"]          = nlopt::GN_ORIG_DIRECT_L;
			algos["GD_STOGO"]                  = nlopt::GD_STOGO;
			algos["GD_STOGO_RAND"]             = nlopt::GD_STOGO_RAND;
			algos["LD_LBFGS_NOCEDAL"]          = nlopt::LD_LBFGS_NOCEDAL;
			algos["LD_LBFGS"]                  = nlopt::LD_LBFGS;
			algos["LN_PRAXIS"]                 = nlopt::LN_PRAXIS;
			algos["LD_VAR1"]                   = nlopt::LD_VAR1;
			algos["LD_VAR2"]                   = nlopt::LD_VAR2;
			algos["LD_TNEWTON"]                = nlopt::LD_TNEWTON;
			algos["LD_TNEWTON_RESTART"]        = nlopt::LD_TNEWTON_RESTART;
			algos["LD_TNEWTON_PRECOND"]        = nlopt::LD_TNEWTON_PRECOND;
			algos["LD_TNEWTON_PRECOND_RESTART"]= nlopt::LD_TNEWTON_PRECOND_RESTART;
			algos["GN_CRS2_LM"]                = nlopt::GN_CRS2_LM;
			algos["GN_MLSL"]                   = nlopt::GN_MLSL;
			algos["GD_MLSL"]                   = nlopt::GD_MLSL;
			algos["GN_MLSL_LDS"]               = nlopt::GN_MLSL_LDS;
			algos["GD_MLSL_LDS"]               = nlopt::GD_MLSL_LDS;
			algos["LD_MMA"]                    = nlopt::LD_MMA;
			algos["LN_COBYLA"]                 = nlopt::LN_COBYLA;
			algos["LN_NEWUOA"]                 = nlopt::LN_NEWUOA;
			algos["LN_NEWUOA_BOUND"]           = nlopt::LN_NEWUOA_BOUND;
			algos["LN_NELDERMEAD"]             = nlopt::LN_NELDERMEAD;
			algos["LN_SBPLX"]                  = nlopt::LN_SBPLX;
			algos["LN_AUGLAG"]                 = nlopt::LN_AUGLAG;
			algos["LD_AUGLAG"]                 = nlopt::LD_AUGLAG;
			algos["LN_AUGLAG_EQ"]              = nlopt::LN_AUGLAG_EQ;
			algos["LD_AUGLAG_EQ"]              = nlopt::LD_AUGLAG_EQ;
			algos["LN_BOBYQA"]                 = nlopt::LN_BOBYQA;
			algos["GN_ISRES"]                  = nlopt::GN_ISRES;
		}
		return algos;
	}

	/// Create a vector of strings identifying all NLopt algorithms available. Pass prefix "LN" for local derivative-free, for instance
	inline std::vector<std::string> algorithm_names(const std::string& prefix="")
	{
		auto &algos(nlopt_algorithms());
		std::vector<std::string> list;
		for (auto it=algos.begin();it!=algos.end();++it) {
			bool prefix_ok=prefix.size()<=it->first.size();
			for (int i=0;i<(int)prefix.size() && prefix_ok;i++)
				prefix_ok&=prefix[i]==it->first[i];
			if (prefix_ok) list.push_back(it->first);
		}
		return list;
	}

	/// Get an NLOpt algorithm enum from a string.  Throws for values not contained in NLOptUtils::nlopt_algorithms
	inline nlopt::algorithm nlopt_algorithm_by_str(const std::string& algo)
	{
		auto it=nlopt_algorithms().find(algo);
		if (it==nlopt_algorithms().end())
			throw std::domain_error("nlopt_algorithm_by_str invalid string or unkown algorithm." );
		return it->second;
	}

	/// Return a human readable account of NLOpt's status after an optimization (attempt).
	inline const std::string& nlopts_exit_status(int status)
	{
		std::string exception="NLopt: Exception Occured.";
		static std::map<int,std::string> nlopts_exit_stati;
		if (nlopts_exit_stati.empty())
		{
			nlopts_exit_stati[NLOPT_SUCCESS]          = "Generic success.";
			nlopts_exit_stati[NLOPT_STOPVAL_REACHED]  = "stopval (above) was reached.";
			nlopts_exit_stati[NLOPT_FTOL_REACHED]     = "ftol_rel or ftol_abs (above) was reached.";
			nlopts_exit_stati[NLOPT_XTOL_REACHED]     = "xtol_rel or xtol_abs (above) was reached.";
			nlopts_exit_stati[NLOPT_MAXEVAL_REACHED]  = "maxeval (above) was reached.";
			nlopts_exit_stati[NLOPT_MAXTIME_REACHED]  = "maxtime (above) was reached.";
			nlopts_exit_stati[NLOPT_FAILURE]          = "Generic failure.";
			nlopts_exit_stati[NLOPT_INVALID_ARGS]     = "Invalid arguments.";
			nlopts_exit_stati[NLOPT_OUT_OF_MEMORY]    = "Ran out of memory.";
			nlopts_exit_stati[NLOPT_ROUNDOFF_LIMITED] = "Halted because roundoff errors limited progress.";
			nlopts_exit_stati[NLOPT_FORCED_STOP]      = "Forced Termination.";
		}
		auto ret=nlopts_exit_stati.find(status);
		return (ret==nlopts_exit_stati.end())?exception:ret->second;
	}

	/// ParameterModel must support static ParameterNames() member. See also Parametrization::AbstractModel.
	struct OptimizerSettings : public GetSetGui::Configurable
	{
		struct Tolerance {
			double parameter_abs=0.01;
			double objective_abs=0.0001;			
			double parameter_rel=0.01;
			double objective_rel=0.01;
		} tolerance;

		std::vector<std::string> algorithm_choices;
		std::string algorithm;
		
		OptimizerSettings(const std::vector<std::string>& possible_algorithms=UtilsNLOpt::algorithm_names()) : algorithm_choices(possible_algorithms) {}

		/// Utility GUI function for LibGetSet. Declate parameters and define default values.
		void gui_declare_section(const GetSetGui::Section& section)
		{
			GetSet<double>("Tolerance/Relative Parameters", section, tolerance.parameter_rel);
			GetSet<double>("Tolerance/Relative Objective" , section, tolerance.objective_rel);
			GetSet<double>("Tolerance/Absolute Parameters", section, tolerance.parameter_abs);
			GetSet<double>("Tolerance/Absolute Objective" , section, tolerance.objective_abs);
			section.subsection("Tolerance").setGrouped();
			GetSetGui::Enum("Algorithm", section)
				.setChoices(algorithm_choices);
		}

		/// Utility GUI function for LibGetSet. Get current values.
		void gui_retreive_section(const GetSetGui::Section& section)
		{
			tolerance.parameter_rel=GetSet<double>("Tolerance/Relative Parameters" ,section);
			tolerance.objective_rel=GetSet<double>("Tolerance/Relative Objective"  ,section);
			tolerance.parameter_abs=GetSet<double>("Tolerance/Absolute Parameters" ,section);
			tolerance.objective_abs=GetSet<double>("Tolerance/Absolute Objective"  ,section);
			algorithm=GetSet<>("Algorithm",section);
		}

	};

	/// NLOpt Optimization Object
	class Optimizer : public LibOpterix::AbstractOptimizer
	{
		nlopt::opt opt;
		bool cancel_clicked;
		// Cast this pointer and call AbstractOptimizationProblem's evaluate
		static double objective_callback(unsigned n, const double *x, double *grad, void *ptr_this)
		{
			double cost=0;
			std::vector<double> x_stl(x,x+n);
			// Get AbstractOptimizer object ("this")
			Optimizer *optimizer=static_cast<Optimizer*>(ptr_this);
			// Check if user has asked for termination
			if (optimizer->cancel_clicked) optimizer->opt.force_stop();
			// Notify progress update (of unkown percentage)
			if (optimizer->progress) optimizer->progress->progressUpdate(-1);
			// Evaluate cost function (with or without gradient)
			if (grad)
			{
				std::vector<double> grad_stl(n,0.0);
				cost=optimizer->abstract_objective.evaluate_gradient(x_stl,grad_stl);
				for (int i=0;i<grad_stl.size();i++) grad[i]=grad_stl[i];
			}
			else
				cost=optimizer->abstract_objective.evaluate(x_stl);
			// Notify user of result of cost function evaluation
			if (optimizer->user_callback) optimizer->user_callback(cost,&optimizer->abstract_objective);
			return cost;
		}

	public:
		Optimizer(LibOpterix::AbstractOptimizationProblem& _objective, const OptimizerSettings& opt_settings,
			const std::vector<double>& bounds_low, const std::vector<double>& bounds_hi )
			: AbstractOptimizer(_objective)
			, opt(nlopt_algorithm_by_str(opt_settings.algorithm),abstract_objective.abstract_model.numberOfParametersActive())
		{
			// Call-back objective function
			opt.set_min_objective(objective_callback,(void*)this);
			// Parameter Settings
			opt.set_lower_bounds(bounds_low);
			opt.set_upper_bounds(bounds_hi );
			// Optimization Settings
			opt.set_ftol_rel(opt_settings.tolerance.objective_rel);
			opt.set_ftol_rel(opt_settings.tolerance.parameter_rel);
			opt.set_ftol_abs(opt_settings.tolerance.objective_abs);
			opt.set_ftol_abs(opt_settings.tolerance.parameter_abs);

		}

		virtual bool optimize(GetSetGui::ProgressInterface& progress)
		{
			// Notify user of starting progress
			cancel_clicked=false;
			progress.progressStart("Optinizing...",std::string("NLOpt optimization with ")+opt.get_algorithm_name(),-1,&cancel_clicked);
			// Start optimization
			std::vector<double> x(opt.get_dimension());
			double minf;
			int result=INT_MIN;
			try {
				result=opt.optimize(x,minf);
			} catch (const std::exception& e) {
				progress.warn("Exception occured", e.what());
			}
			// Finish up and check status.
			progress.progressEnd();
			if (result==NLOPT_SUCCESS)
				return true;
			progress.warn("Optimization Failed", nlopts_exit_status(result));
			return false;
		}

	};

} // namespace UtilsNLOpt
	
#endif // nlopt_utils_hxx
