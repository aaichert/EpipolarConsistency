#ifndef _ecc_single_image_motion_hxx
#define _ecc_single_image_motion_hxx

#include <LibOpterix/Opterix.hxx>

#include <LibEpipolarConsistency/EpipolarConsistency.h>

#include <numeric>

namespace EpipolarConsistency {

	// Epipolar consistency metric for changes on one projection matrix.
	class SingleImageMotion : public LibOpterix::OptimizationProblem<Geometry::ProjectionMatrix>
	{
		std::vector<Geometry::ProjectionMatrix>                 Ps;          //< Projection Matrices. First is input image.
		std::vector<EpipolarConsistency::RadonIntermediate*>    dtrs;        //< Radon Intermediate functions. First is input image.
		int                                                     input_index; //< Index of projection matric being optimized over.

		EpipolarConsistency::MetricRadonIntermediate            *ecc;        //< The object which computes ECC on the GPU
		std::vector<Eigen::Vector4i>                            indices;     //< The pairs of views used for evaluation
		std::vector<float>                                      tmp_results; //< The most pairwise results is the last evalutaion.

		SingleImageMotion(const SingleImageMotion&);

	public:
		/// Create a SingleImageMotion object from a set of projections and an index
		SingleImageMotion(
				LibOpterix::ParameterModel<Geometry::ProjectionMatrix>& _parameter_model,
				std::vector<Geometry::ProjectionMatrix> _Ps,
				std::vector<EpipolarConsistency::RadonIntermediate*> _dtrs,
				int _input_index=0
			)
			: LibOpterix::OptimizationProblem<Geometry::ProjectionMatrix>(_parameter_model)
			, Ps(_Ps) 
			, dtrs(_dtrs)
			, input_index(_input_index)
			, ecc(new EpipolarConsistency::MetricRadonIntermediate(Ps,dtrs))
		{
			int n=(int)_dtrs.size();
			for (int i=0;i<n;i++) if (i!=input_index)
				indices.push_back(Eigen::Vector4i(input_index,i,input_index,i));
		}

		~SingleImageMotion() {
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
				out=tmp_results.data();
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

#endif // _ecc_single_image_motion_hxx
