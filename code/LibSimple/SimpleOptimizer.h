#ifndef SIMPLE_OPTIMIZER_PB
#define SIMPLE_OPTIMIZER_PB

/** @brief	Powell-Brent Optimizer based on Numerical Recepies
 *  @author	Andre Aichert
 */


#include <vector>

// Simple Optimization of a function R^n->R
namespace Simple
{

	class Optimizer
	{
	public:

		Optimizer(double step=1, double ftol=0.01, double * _weigths=0x0) // 2do add max its
			:m_step(step), m_ftol(ftol), weigths(_weigths)
		{}

		double m_step;	// scaling of initial search directions
		double m_ftol;	// Tolerance in objective function value
		
		int minimize(std::vector<double>& x, double (*objFun)(double*) )
		{
			if (x.empty()) return -1;
			std::vector<double> delta(x.size());
			if (weigths)
				for (int i=0;i<(int)delta.size(); i++)
					delta[i]=m_step*weigths[i];
			else
				for (int i=0;i<(int)delta.size(); i++)
					delta[i]=m_step;
			int iterNr=20;
			double fret;
			return powell(&(x[0]), &(delta[0]), (int)x.size(), m_ftol, &iterNr, &fret, objFun);
		}

	private:
		static int powell(
			double *p,		// solution
			double *delta,	// step (for example vector of all ones)
			int parNr,		// problem size
			double ftol,	// output tolerance
			int *iterNr,	// output iterations
			double *fret,	// returned function value
			double (*_fun)(double*) );
			double * weigths;
	};
	
} // namespace Simple

#endif // SIMPLE_OPTIMIZER_PB
