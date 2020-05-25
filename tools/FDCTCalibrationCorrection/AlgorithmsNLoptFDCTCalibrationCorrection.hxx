// choose nl_opt algortithm 
// Lina Felsner 10.8.2016

#include <nlopt.hpp>

#include <iostream>
#include <map>
#include <stdexcept>

namespace NLOptUtil {

class Algorithm
{
private:
	static const std::map<std::string, nlopt::algorithm>& nlopt_algorithms()
	{
		static std::map<std::string, nlopt::algorithm> algos;
		if (algos.empty())
		{
			algos["GN_DIRECT"] =                  nlopt::GN_DIRECT;
			algos["GN_DIRECT_L"] =                nlopt::GN_DIRECT_L;
			algos["GN_DIRECT_L_RAND"] =           nlopt::GN_DIRECT_L_RAND;
			algos["GN_DIRECT_NOSCAL"] =           nlopt::GN_DIRECT_NOSCAL;
			algos["GN_DIRECT_L_NOSCAL"] =         nlopt::GN_DIRECT_L_NOSCAL;
			algos["GN_DIRECT_L_RAND_NOSCAL"] =    nlopt::GN_DIRECT_L_RAND_NOSCAL;
			algos["GN_ORIG_DIRECT"] =             nlopt::GN_ORIG_DIRECT;
			algos["GN_ORIG_DIRECT_L"] =           nlopt::GN_ORIG_DIRECT_L;
			algos["GD_STOGO"] =                   nlopt::GD_STOGO;
			algos["GD_STOGO_RAND"] =              nlopt::GD_STOGO_RAND;
			algos["LD_LBFGS_NOCEDAL"] =           nlopt::LD_LBFGS_NOCEDAL;
			algos["LD_LBFGS"] =                   nlopt::LD_LBFGS;
			algos["LN_PRAXIS"] =                  nlopt::LN_PRAXIS;
			algos["LD_VAR1"] =                    nlopt::LD_VAR1;
			algos["LD_VAR2"] =                    nlopt::LD_VAR2;
			algos["LD_TNEWTON"] =                 nlopt::LD_TNEWTON;
			algos["LD_TNEWTON_RESTART"] =         nlopt::LD_TNEWTON_RESTART;
			algos["LD_TNEWTON_PRECOND"] =         nlopt::LD_TNEWTON_PRECOND;
			algos["LD_TNEWTON_PRECOND_RESTART"] = nlopt::LD_TNEWTON_PRECOND_RESTART;
			algos["GN_CRS2_LM"] =                 nlopt::GN_CRS2_LM;
			algos["GN_MLSL"] =                    nlopt::GN_MLSL;
			algos["GD_MLSL"] =                    nlopt::GD_MLSL;
			algos["GN_MLSL_LDS"] =                nlopt::GN_MLSL_LDS;
			algos["GD_MLSL_LDS"] =                nlopt::GD_MLSL_LDS;
			algos["LD_MMA"] =                     nlopt::LD_MMA;
			algos["LN_COBYLA"] =                  nlopt::LN_COBYLA;
			algos["LN_NEWUOA"] =                  nlopt::LN_NEWUOA;
			algos["LN_NEWUOA_BOUND"] =            nlopt::LN_NEWUOA_BOUND;
			algos["LN_NELDERMEAD"] =              nlopt::LN_NELDERMEAD;
			algos["LN_SBPLX"] =                   nlopt::LN_SBPLX;
			algos["LN_AUGLAG"] =                  nlopt::LN_AUGLAG;
			algos["LD_AUGLAG"] =                  nlopt::LD_AUGLAG;
			algos["LN_AUGLAG_EQ"] =               nlopt::LN_AUGLAG_EQ;
			algos["LD_AUGLAG_EQ"] =               nlopt::LD_AUGLAG_EQ;
			algos["LN_BOBYQA"] =                  nlopt::LN_BOBYQA;
			algos["GN_ISRES"] =                   nlopt::GN_ISRES;
		}
		return algos;
	}	
	


public:

	static nlopt::algorithm nlopt_algorithm_by_str(const std::string& algo)
	{
		auto it = nlopt_algorithms().find(algo);
		if (it == nlopt_algorithms().end())
			throw std::invalid_argument("nlopt_algorithm_by_str unknown algorithm." );
		return it->second;
	}

	static const std::string& nlopts_exit_status(int status)
	{
		std::string exception = "NLopt: Exception Occured.";
		static std::map<int, std::string> nlopts_exit_stati;
		if (nlopts_exit_stati.empty())
		{
			nlopts_exit_stati[NLOPT_SUCCESS] =          "Generic success.";
			nlopts_exit_stati[NLOPT_STOPVAL_REACHED] =  "stopval (above) was reached.";
			nlopts_exit_stati[NLOPT_FTOL_REACHED] =     "ftol_rel or ftol_abs (above) was reached.";
			nlopts_exit_stati[NLOPT_XTOL_REACHED] =     "xtol_rel or xtol_abs (above) was reached.";
			nlopts_exit_stati[NLOPT_MAXEVAL_REACHED] =  "maxeval (above) was reached.";
			nlopts_exit_stati[NLOPT_MAXTIME_REACHED] =  "maxtime (above) was reached.";
			nlopts_exit_stati[NLOPT_FAILURE] =          "Generic failure.";
			nlopts_exit_stati[NLOPT_INVALID_ARGS] =     "Invalid arguments.";
			nlopts_exit_stati[NLOPT_OUT_OF_MEMORY] =    "Ran out of memory.";
			nlopts_exit_stati[NLOPT_ROUNDOFF_LIMITED] = "Halted because roundoff errors limited progress.";
			nlopts_exit_stati[NLOPT_FORCED_STOP] =      "Forced Termination.";
		}
		auto ret = nlopts_exit_stati.find(status);
		return (ret == nlopts_exit_stati.end()) ? exception : ret->second;
	}

};

} // namespace NLOptUtil