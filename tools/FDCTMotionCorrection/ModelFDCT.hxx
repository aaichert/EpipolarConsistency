// Parametrization of a projective 2D or 3D transformation
#include <LibProjectiveGeometry/Models/ModelCameraSimilarity2D3D.hxx>

namespace Geometry
{
	/// Parametrization of motion correction for FDCT
	struct ModelFDCT
	{

		ModelCameraSimilarity2D3D	stencil;		//< All projection matrices will abide to this model
		int							n_active;		//< Number of active parameters in stencil
		int							n_proj;			//< Number of projections

		std::vector<double>		params;			//< Raw parameter vector (remains constant)
		std::vector<double>		params_delta;	//< Raw parameter vector plus those currently being changed

		std::vector<Geometry::ProjectionMatrix> trajectory; //< current estimate of the trajectory

		/// Parametrize FDCT trajectory via projective transformation of space and image
		ModelFDCT(ModelCameraSimilarity2D3D _stencil)
			: stencil(_stencil)
			, n_active((int)stencil.getActiveParamIndices().size())
		{}

		/// Apply parameter vector for view i. If view<0, all parameters will be updated.
		std::vector<Geometry::ProjectionMatrix>& applyModel(int view, const double *delta, const std::vector<Geometry::ProjectionMatrix>& Ps)
		{
			// Make sure we have work space
			n_proj=(int)Ps.size();
			if (trajectory.size()!=n_proj) trajectory.resize(n_proj);
			if (params.size()!=n_proj*n_active) params=std::vector<double>(n_proj*n_active,0.0);
			if (params_delta.size()!=n_proj*n_active) params_delta=std::vector<double>(n_proj*n_active,0.0);

			// Copy delta to params_delta
			#pragma omp parallel for
			for (int i=0;i<n_proj*n_active;i++)
				params_delta[i]=0;
			if (delta)
			{
				if (view<0)
					#pragma omp parallel for
					for (int i=0;i<n_proj*n_active;i++)
						params_delta[i]=delta[i];
				else
					#pragma omp parallel for
					for (int i=0;i<n_active;i++)
						params_delta[view*n_active+i]=delta[i];
					
			}
			#pragma omp parallel for
			for (int i=0;i<n_proj*n_active;i++)
				params_delta[i]+=params[i];
			// Apply parameter model
			#pragma omp parallel for
			for (int i=0;i<(int)n_proj;i++)
			{
				ModelCameraSimilarity2D3D HTi=stencil;
				HTi.expand(&params_delta[i*n_active]);
				trajectory[i]=HTi.transform(Ps[i]);
			}
			return trajectory;
		}

	};

} // Geometry
