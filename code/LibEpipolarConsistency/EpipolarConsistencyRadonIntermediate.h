#ifndef __epipolar_consistency_radon_intermediate
#define __epipolar_consistency_radon_intermediate
// Created by A. Aichert on Thu Nov 21st 2013
// Class to compute Epipolar Consistency given ProjectionMatrices and RadonIntermediates

#include "EpipolarConsistency.h"

#include <LibEpipolarConsistency/RadonIntermediate.h>

#include <set>

// Predeclaration of some CUDA stuff
namespace UtilsCuda {
	template <typename T> class BindlessTexture2D;
	template <typename T> class MemoryBlock;
} // namespace UtilsCuda

namespace EpipolarConsistency
{
	/// Compute Epipolar Consistency on the GPU
	class MetricRadonIntermediate : public Metric {
		/// Radon intermediate functions.
		std::vector<RadonIntermediate*> dtrs;

		/// Use correlation coefficient or sum of squared differences to compare signals
		bool use_corr;

	public:
		MetricRadonIntermediate();

		MetricRadonIntermediate(const std::vector<ProjectionMatrix>& Ps, const std::vector<RadonIntermediate*>& _dtrs);

		~MetricRadonIntermediate();

		/// Deprecated.
		MetricRadonIntermediate& setdKappa(float _dkappa)
		{
			dkappa=_dkappa;
			return *this;
		}

		/// Tell Metric to compute correlation instead of SSD
		MetricRadonIntermediate& useCorrelation(bool corr=true);

		/// Move Radon derivetaives to texture memory and store reference here. DO NOT delete or change _dtrs during lifetime of Metric.
		MetricRadonIntermediate& setRadonIntermediates(const std::vector<RadonIntermediate*>& _dtrs);
	
		/// Access Radon intermediate functions (for visualization and debugging)
		const std::vector<RadonIntermediate*>& getRadonIntermediates() const;
	
		/// Set parameters for computation of Radon intermediate functions in setProjectionImages(...)
		Metric&  setRadonIntermediateBinning(int num_bins_per_180_deg, int num_bins_per_image_diagonal);

		/// Set projections images from single-channel 2D float textures. Must call setRadonIntermediateBinning first
		virtual Metric& setProjectionImages(const std::vector<UtilsCuda::BindlessTexture2D<float>*>& Is);

		/// Compute null space and pseudoinverse of projection matrices and convert to float (GPU is only faster for very large problems)
		virtual Metric& setProjectionMatrices(const std::vector<ProjectionMatrix>& Ps);

		/// Evaluates metric without any transformation of the geometry. Out is n*n and mean is returned.
		virtual double evaluate(float * out=0x0);

		/// The number of projections. The number of evaluations will be n*(n-1)/2
		virtual int  getNumberOfProjetions() {return (int) Ps.size();}

		/// Evaluates metric for just specific view
		double evaluate(const std::set<int>& views, float * _out=0x0);

		/// Evaluates metric without any transformation of the geometry. Indices addresses (P0,P1,dtr0,dtr1)
		double evaluate(const std::vector<Eigen::Vector4i>& _indices, float * _out);

		/// Evaluate for just tow images i and j and optionally also return redundant values (visualization only - slow, not using GPU)
		virtual double evaluateForImagePair(int i, int j,
			std::vector<float> *redundant_samples0=0x0, std::vector<float> *redundant_samples1=0x0,
			std::vector<float> *kappas=0x0);

		/// Evaluate for just tow images i and j and  return redundant values and sample locations in the Radon transforms (visualization only - slow, not using GPU)
		double evaluateForImagePair(int i, int j,
			std::vector<float> *redundant_samples0, std::vector<float> *redundant_samples1,
			std::vector<float> *kappas,
			std::vector<std::pair<float,float> > *radon_samples0,
			std::vector<std::pair<float,float> > *radon_samples1 );

	protected:
		MetricRadonIntermediate(const MetricRadonIntermediate&);
		
		UtilsCuda::MemoryBlock<float>	*Cs;		//< Source positions
		UtilsCuda::MemoryBlock<float>	*PinvTs;	//< Inverse transpose of projection matrices
		UtilsCuda::MemoryBlock<char>	*tex_dtrs;	//< Radon Derivative textures on GPU
		UtilsCuda::MemoryBlock<float>	*K01s;		//< Mappings from plane angle to epipolar lines
		UtilsCuda::MemoryBlock<float>	*out;		//< Results
		UtilsCuda::MemoryBlock<float>	*corr_out;	//< Optional intermediate storagy for correlation computation 

		UtilsCuda::MemoryBlock<int>		*indices;	//< Optional selection of specific indices.
		int								n_pairs;	//< Number of indices for which GPU memory has been allocated

		int n_t;				//< Size of Radon transform line distance (y) direction
		int n_alpha;			//< Size of Radon transform angular (x) direction
		float step_t;			//< Line distance spacing of Radon transform
		float step_alpha;		//< Angular spacing of Radon transform
		
		double focal_length_px; //< An estimate of the focal length to estimate dkappa, if required.

		bool m_isDerivative;	//< If false, normal Radon transform (or ramp-filtered), else its derivative

	};

} // namespace EpipolarConsistency

#endif // __epipolar_consistency_radon_intermediate
