#ifndef _ecc_motioncompenstation_hxx
#define _ecc_motioncompenstation_hxx

#include <LibOpterix/Opterix.hxx>

#include <LibEpipolarConsistency/EpipolarConsistency.h>

#include <numeric>

namespace EpipolarConsistency {

	// Epipolar consistency metric for changes on one projection matrix.
	class FDCTMoCo : public LibOpterix::OptimizationProblem<std::vector<Geometry::ProjectionMatrix> >
	{
		std::vector<Geometry::ProjectionMatrix>                 Ps;          //< Initial Guess of Projection Matrices.
		std::vector<EpipolarConsistency::RadonIntermediate*>    dtrs;        //< Radon Intermediate functions.

		EpipolarConsistency::MetricRadonIntermediate            *ecc;        //< The object which computes ECC on the GPU
		NRRD::Image<float>                                      cost_image;  //< Pairwise results for initial guess (lower triangular part) and last evaluation (upper triangular part)
																			 
		FDCTMoCo(const FDCTMoCo&);

	public:
		/// Create a FDCTMoCo object from a set of projections
		FDCTMoCo(
				LibOpterix::ParameterModel<Geometry::ProjectionMatrix>& _parameter_model,
				std::vector<Geometry::ProjectionMatrix> _Ps,
				std::vector<EpipolarConsistency::RadonIntermediate*> _dtrs,
				int _input_index=0
			)
			: LibOpterix::OptimizationProblem<Geometry::ProjectionMatrix>(_parameter_model)
			, Ps(_Ps) 
			, dtrs(_dtrs)
			, ecc(new EpipolarConsistency::MetricRadonIntermediate(Ps,dtrs))
		{
			int n=(int)_dtrs.size();
			for (int i=0;i<n;i++) if (i!=input_index)
				indices.push_back(Eigen::Vector4i(input_index,i,input_index,i));
		}

		~FDCTMoCo() {
			delete ecc;
		}

		EpipolarConsistency::MetricRadonIntermediate& getMetricPtr() { return *ecc; }

		/// Access to results of most recent eveluation.
		const std::vector<float>& getTemporaryResults()
		{
			return cost_image;
		}

		/// (re-)evaluate epipolar consistency for input image. Optional n*n image out contains metric for all pairs before and after optimization.
		double evaluate(float *out=0x0)
		{
			/// An optional temporary vector
			if (!out) {
				cost_image.set(dtrs.size(),dtrs.size());
				int l=cost_image.length();
				#pragma omp parallel for
				for (int i=0;i<l;i++) cost_image[i]=0;
				out=cost_image;
			}


			return ecc->evaluate(); // Should be same as ecc->evaluate(indices, out); except some constant part is added.


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
			Ps[input_index]=P_input;
			if (!ecc) ecc=new EpipolarConsistency::MetricRadonIntermediate(Ps,dtrs);
			else ecc->setProjectionMatrices(Ps);
			return evaluate();
		}

	};

} // EpipolarConsistency

#endif // _ecc_motioncompenstation_hxx
