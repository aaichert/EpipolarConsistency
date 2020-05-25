// LibOpterix.
// +                                          `@
// `@                                        .@#
//  @@.                                  ,+@@#  
//    @@@                            .@@@@+.  @,
//      ,@@@@@@@@@@#',              @@@.  `@@@.
//     @        `,;#@@@@@@;    ,,  ++', :@@+
//      #@.             +@@@.@  +'    @@:,
//         :+@@@@@@@@@@`   @           @@
//        '           ,@@+@`           @@ 
//                    ,#@@@@          `@
//                       @@           @  
//                       :@@        O  P  T  E  R  I  X
//                         @@@+;'@: 
// A c++ library allowing non-linear optimization to easily exchange parametrizations and optimization algorthms/libraries.
// To use this library, you need to define two classes, one deriving from OptimizationProblem, and another deriving from AbstractOptimizer.
// This library includes a simple grid-search and Powell optimizer and optional wrappers for NLopt (others pending).

#ifndef __opterix_hxx
#define __opterix_hxx

#include <Utils/ProgressInterface.hxx>
#include <LibOpterix/ParameterModel.hxx>

namespace LibOpterix {

	/// Interface for an objective function
	struct AbstractOptimizationProblem
	{
		AbstractParameterModel& abstract_model;
		AbstractOptimizationProblem(AbstractParameterModel& _abstract_model)
			: abstract_model(_abstract_model)
		{}

		// Compute cost function value given a parameter vector
		virtual double evaluate(const std::vector<double>& x) =0;

		/// Compute gradient and cost function value.
		virtual double evaluate_gradient(const std::vector<double>& x, std::vector<double>& gradient)
		{
			throw std::logic_error("AbstractObjectiveFunction: Gradient computation is not implemented for this optimization problem.");
		}
	};

	/// The actual optimization problem, which uses a specific parametrization of ParametrizedObject.
	template <typename ParametrizedObject>
	struct OptimizationProblem : public AbstractOptimizationProblem
	{
		ParameterModel<ParametrizedObject>& model;
		OptimizationProblem(ParameterModel<ParametrizedObject>& _model)
			: AbstractOptimizationProblem(_model)
			, model(_model)
		{}

		/// Compute cost function value from ParametrizedObject
		virtual double evaluate(const ParametrizedObject& obj) =0;

		/// Compute cost function value given a parameter vector
		virtual double evaluate(const std::vector<double>& x)
		{
			model.expand(&x[0]);
			return evaluate(model.getInstance());
		}
	};

	/// To use this library, you need to define two classes, one deriving from OptimizationProblem, and another deriving from AbstractOptimizer.
	struct AbstractOptimizer
	{
	public:
		// Simple callback mechanism for a notification after each cost-function evaluation
		typedef void (*NotifyUpdate)(double current_cost, AbstractOptimizationProblem* problem);

		// Optimization problem
		AbstractOptimizationProblem& abstract_objective;
		AbstractOptimizer(AbstractOptimizationProblem& _abstract_objective)
			: abstract_objective(_abstract_objective)
			, user_callback(0x0)
			, progress(0x0)
		{}
		
		/// Perform optimization over parameters in AbstractOptimizationProblem.
		virtual bool optimize(GetSetGui::ProgressInterface&) =0;
		
		/// Optionally ask optimizer to notify called after each cost function evaluation.
		AbstractOptimizer& setCallback(NotifyUpdate _callback)
		{
			user_callback=_callback;
			return *this;
		}

	protected:
		NotifyUpdate user_callback;
		GetSetGui::ProgressInterface *progress;
	};

} // namspace Optimizationx

#endif // __opterix_hxx
