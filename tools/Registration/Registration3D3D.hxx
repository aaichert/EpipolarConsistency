#ifndef _ecc_registration_3d_3d_
#define _ecc_registration_3d_3d_

#include <LibOpterix/Opterix.hxx>

#include <LibEpipolarConsistency/EpipolarConsistencyRadonIntermediate.h>

#include <NRRD/nrrd_image.hxx>
#include <numeric>

namespace EpipolarConsistency {

	// Epipolar consistency metric for changes on one projection matrix.
	class Registration3D3D : public LibOpterix::OptimizationProblem<Geometry::RP3Homography>
	{
		std::vector<Geometry::ProjectionMatrix>					Ps;		//< Projection Matrices. Source Projection matrices come first.
		std::vector<EpipolarConsistency::RadonIntermediate*>	dtrs;	//< Radon Intermediate functions. First is input image.

		bool use_cc;

		int n_source;													//< Number of projections of the registration source.
		int n_target;													//< Number of projections of the registration target.

		std::vector<Eigen::Vector4i>    indices;						//< The pairs of views used for evaluation
		mutable EpipolarConsistency::MetricRadonIntermediate ecc;		//< The object which computes ECC on the GPU
		mutable NRRD::Image<float>      tmp_results;					//< The most recent pairwise results  (source-fast)

		Registration3D3D(const Registration3D3D&);

	public:

		Registration3D3D(
				LibOpterix::ParameterModel<Geometry::RP3Homography>& _parameter_model,
				bool _use_cc,
				std::vector<Geometry::ProjectionMatrix> _Ps_source,
				std::vector<EpipolarConsistency::RadonIntermediate*> _dtrs_source,
				std::vector<Geometry::ProjectionMatrix> _Ps_target,
				std::vector<EpipolarConsistency::RadonIntermediate*> _dtrs_target
			)
			: LibOpterix::OptimizationProblem<Geometry::RP3Homography>(_parameter_model)
		{


			// Number of input projections
			n_source=(int)_dtrs_source.size();
			n_target=(int)_dtrs_target.size();
			int n=n_target+n_source;
			Ps.resize(n);
			dtrs.resize(n);
			// Copy input data into common vectors
			for (int i=0;i<n_source;i++) {
				Ps[i]=_Ps_source[i];
				dtrs[i]=_dtrs_source[i];
			}
			for (int j=0;j<n_target;j++) {
				Ps  [n_source+j]=_Ps_target  [j];
				dtrs[n_source+j]=_dtrs_target[j];
			}
			// All pairs with one source and one target projection (stored source-index-fast)
			int N=n_source*n_target;
			indices.resize(N);
			for (int j=0;j<n_target;j++)
				for (int i=0;i<n_source;i++)
					indices[i*n_source+j]=Eigen::Vector4i(i,n_source+j,i,n_source+j);
			// Create metric
			ecc.setProjectionMatrices(Ps);
			ecc.setRadonIntermediates(dtrs);
			// Metric
			use_cc=_use_cc;
			if (use_cc)
				ecc.useCorrelation();
		}

		EpipolarConsistency::MetricRadonIntermediate& getMetricPtr() const { return ecc; }

		/// Access to results of most recent eveluation. See also: getIndices()
		const NRRD::ImageView<float>& getTemporaryResults() const
		{
			if (!tmp_results)
				tmp_results.set(n_source,n_target);
			return tmp_results;
		}

		/// Access to the pairs of images, which are used for evaluation. See also: getTemporaryResults()
		const std::vector<Eigen::Vector4i>& getIndices() const
		{
			return indices;
		}

		/// (re-)evaluate epipolar consistency for input image. Optional n-1 vector out contains metric for all pairs.
		double evaluate(float *out=0x0)
		{
			/// An optional temporary vector
			if (!out)  out=getTemporaryResults();
			ecc.evaluate(indices, out);
			int n=n_source*n_target;
			double sum=0;
			for (int i=0;i<n;i++)
				sum+=out[i]/n;
			return sum;
		}

		/// Evaluate cost function with T_input between source and target.
		virtual double evaluate(const Geometry::RP3Homography& T_input)
		{
			auto Ps_transformed=Ps;
			for (int i=0;i<n_source;i++)
				Ps_transformed[i]=Ps[i]*T_input;
			ecc.setProjectionMatrices(Ps_transformed);
			return evaluate();
		}

	};

} // EpipolarConsistency

#endif // _ecc_registration_3d_3d_
