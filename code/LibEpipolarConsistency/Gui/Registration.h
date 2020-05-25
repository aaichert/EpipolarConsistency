#ifndef _ecc_motioncompenstation_hxx
#define _ecc_motioncompenstation_hxx

#include <LibOpterix/Opterix.hxx>

#include <LibEpipolarConsistency/EpipolarConsistency.h>

#include <numeric>

namespace EpipolarConsistency {

	// Epipolar consistency metric for registration of two CTs
	class Registration : public LibOpterix::OptimizationProblem<Geometry::ProjectionMatrix>
	{
		std::vector<Geometry::ProjectionMatrix>					Ps;		//< Projection Matrices. First is input image.
		std::vector<EpipolarConsistency::RadonIntermediate*>	dtrs;	//< Radon Intermediate functions. First is input image.

		EpipolarConsistency::MetricRadonIntermediate *ecc;				//< The object which computes ECC on the GPU
		std::vector<Eigen::Vector4i> indices;							//< The pairs of views used for evaluation
		std::vector<float>           tmp_results;						//< The most pairwise results is the last evalutaion.

		Registration(const Registration&);

	public:

		Registration(
				LibOpterix::ParameterModel<Geometry::ProjectionMatrix>& _parameter_model,
				std::vector<Geometry::ProjectionMatrix> _Ps,
				std::vector<EpipolarConsistency::RadonIntermediate*> _dtrs
			)
			: LibOpterix::OptimizationProblem<Geometry::ProjectionMatrix>(_parameter_model)
			, Ps(_Ps) 
			, dtrs(_dtrs)
			, ecc(new EpipolarConsistency::MetricRadonIntermediate(Ps,dtrs))
		{
			int n=(int)_dtrs.size();
			for (int i=1;i<n;i++)
				indices.push_back(Eigen::Vector4i(0,i,0,i));
		}

		~Registration() {
			delete ecc;
		}

		EpipolarConsistency::MetricRadonIntermediate& getMetricPtr() { return *ecc; }

		/// Access to results of most recent eveluation. See also: getIndices()
		const std::vector<float>& getTemporaryResults()
		{
			return tmp_results;
		}

		/// Access to the pairs of images, which are used for evaluation. See also: getTemporaryResults()
		const std::vector<Eigen::Vector4i>& getIndices()
		{
			return indices;
		}

		/// (re-)evaluate epipolar consistency for input image. Optional n-1 vector out contains metric for all pairs.
		double evaluate(float *out=0x0)
		{
			/// An optional temporary vector
			if (!out) {
				tmp_results.resize(dtrs.size());
				out=&tmp_results[0];
			}
			ecc->evaluate(indices, out);
			int n=(int)dtrs.size()-1;
			double sum=0;
			for (int i=0;i<n;i++)
				sum+=out[i]/n;
			return sum;
		}

		/// Evaluate cost function with P_input for the first image.
		virtual double evaluate(const Geometry::ProjectionMatrix& P_input)
		{
			Ps[0]=P_input;
			if (!ecc) ecc=new EpipolarConsistency::MetricRadonIntermediate(Ps,dtrs);
			else ecc->setProjectionMatrices(Ps);
			return evaluate();
		}

	};

} // EpipolarConsistency

#endif // _ecc_motioncompenstation_hxx
