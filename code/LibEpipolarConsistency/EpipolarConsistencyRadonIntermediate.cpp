#include "EpipolarConsistencyRadonIntermediate.h"

#include <Eigen/Dense>

#include <LibUtilsCuda/CudaMemory.h>
#include <LibUtilsCuda/CudaBindlessTexture.h>

#include <GetSet/ProgressInterface.hxx>

// Eigen is considerbly slower for this particular small problem, so we rely on our own numerical recepies code
#include <LibUtilsCuda/culaut/xprojectionmatrix.hxx>

#include "EpipolarConsistencyCommon.hxx"

// The GPU version
void epipolarConsistency(
	int n_x,				//< Width of original images in pixels
	int n_y,				//< Height of original images in pixels
	int num_dtrs,			//< Number of projection images (=# dtrs)
	char* dtrs_d,			//< DTRs as GPU textures cudaTextureObject_t
	int n_alpha,			//< Number of angular bins (y-direction)
	int n_t,				//< Number of line distance bins (x-direction)
	float step_alpha,		//< Angular spacing of radon transform
	float step_t,			//< Line distance spacing of radon transform
	int num_Ps,				//< Number of projection matrices
	float* Cs_d,			//< Source positions 4 * num_Ps, null spaces of projection matrices
	float* PinvTs_d,		//< Projection Matrix inverse transpose, column major 3x4 * num_Ps
	int num_pairs,			//< Explicit assigment of projection matrices and dtrs. Can be zero.
	int *indices_d,			//< num_indices*4 (P0,P1,dtr0,dtr1). Can be null. If not provided, all n*(n-1)*2 pairs are computed.
	float *K01s_d,			//< num_indices*16 for two 2*3 matrices mapping [cos(kappa sin(kappa)] to lines 0/1
	float *out_d,			//< num_indices result values, unless indices is null. Then nxn matrix. Only sub-diagonal elements are written to.
	float object_radius_mm,	//< Approximate size of object in mm (determines angular range, can be zero)
	float dkappa,			//< Angular distance between epipolar planes (can be zero if object_radius_mm and num_samples are given)
	bool isDerivative,		//< If false, normal Radon transform, else its derivative
	bool use_corr,			//< If null, SSD is used as a similarity measure, else correlation coeff is used.
	float *out_corr_d		//< Temporary memory for weights and NCC computation
	);

namespace EpipolarConsistency
{
	MetricRadonIntermediate::MetricRadonIntermediate()
		: Metric()
		, Cs(new UtilsCuda::MemoryBlock<float>())
		, PinvTs(new UtilsCuda::MemoryBlock<float>())
		, tex_dtrs(new UtilsCuda::MemoryBlock<char>())
		, indices(new UtilsCuda::MemoryBlock<int>())
		, K01s(new UtilsCuda::MemoryBlock<float>())
		, out(new UtilsCuda::MemoryBlock<float>())
		, corr_out(new UtilsCuda::MemoryBlock<float>())
		, use_corr(false)
	{}

	MetricRadonIntermediate::MetricRadonIntermediate(const std::vector<ProjectionMatrix>& Ps, const std::vector<RadonIntermediate*>& _dtrs)
		: Metric()
		, Cs(new UtilsCuda::MemoryBlock<float>())
		, PinvTs(new UtilsCuda::MemoryBlock<float>())
		, tex_dtrs(new UtilsCuda::MemoryBlock<char>())
		, indices(new UtilsCuda::MemoryBlock<int>())
		, K01s(new UtilsCuda::MemoryBlock<float>())
		, out(new UtilsCuda::MemoryBlock<float>()) 
		, corr_out(new UtilsCuda::MemoryBlock<float>())
		, use_corr(false)
	{
		setProjectionMatrices(Ps);
		setRadonIntermediates(_dtrs);
	}

	MetricRadonIntermediate::~MetricRadonIntermediate()
	{
		delete Cs;
		delete PinvTs;
		delete tex_dtrs;
		delete indices;
		delete K01s;
		delete out;
		delete corr_out;
	}

	/// Tell Metric to compute correlation instead of SSD
	MetricRadonIntermediate& MetricRadonIntermediate::useCorrelation(bool corr)
	{
		use_corr=corr;
		return *this;
	}

	/// Move Radon derivataives to texture memory and store reference here. DO NOT delete or change _dtrs during lifetime of MetricRadonIntermediate.
	MetricRadonIntermediate& MetricRadonIntermediate::setRadonIntermediates(const std::vector<RadonIntermediate*>& _dtrs)
	{
		// Store size and parameters of dtrs (assumed to be identical)
		dtrs=_dtrs;
		int n_dtr=(int)_dtrs.size();
		step_alpha=(float)_dtrs[0]->getRadonBinSize(0);
		step_t=(float)_dtrs[0]->getRadonBinSize(1);
		n_alpha=_dtrs[0]->getRadonBinNumber(0);
		n_t=_dtrs[0]->getRadonBinNumber(1);
		n_u=_dtrs[0]->getOriginalImageSize(0);
		n_v=_dtrs[0]->getOriginalImageSize(1);
		m_isDerivative=_dtrs[0]->isDerivative();
		// Download texture ids
		std::vector<cudaTextureObject_t> dtrs_h(n_dtr);
		for (int i=0;i<n_dtr;i++)
			dtrs_h[i]=_dtrs[i]->getTexture()->tex;
		tex_dtrs->allocate(n_dtr*sizeof(cudaTextureObject_t));
		tex_dtrs->download((char*)&dtrs_h[0]);
		return *this;
	}


	/// Access Radon intermediate functions (for visualization and debugging)
	const std::vector<RadonIntermediate*>& MetricRadonIntermediate::getRadonIntermediates() const 
	{
		return dtrs;
	}
	
	//Metric&  MetricRadonIntermediate::setRadonIntermediateBinning(int num_bins_per_180_deg, int num_bins_per_image_diagonal, GetSet::)
	//{
	//	n_alpha =num_bins_per_180_deg;
	//	n_t     =num_bins_per_image_diagonal;
	//}

	Metric& MetricRadonIntermediate::setProjectionImages(const std::vector<UtilsCuda::BindlessTexture2D<float>*>& Is)
	{
		// TODO
		return *this;
	}

	inline float cc(float * xyxxyyxy)
	{
//		return (xyxxyyxy[4]-xyxxyyxy[0]*xyxxyyxy[1])/(sqrt(xyxxyyxy[2]-xyxxyyxy[0]*xyxxyyxy[0])*sqrt(xyxxyyxy[3]-xyxxyyxy[1]*xyxxyyxy[1]));
		return xyxxyyxy[4]/(sqrt(xyxxyyxy[2])*sqrt(xyxxyyxy[3]));
	}

	/// Compute null space and pseudoinverse of projection matrices and convert to float
	Metric& MetricRadonIntermediate::setProjectionMatrices(const std::vector<ProjectionMatrix>& Ps)
	{
		Metric::setProjectionMatrices(Ps);
		// Make sure we have data.
		int n=(int)Ps.size();
		if (n == 0) return *this;
		// Store examplary parameters of first projections matrix, to estimate dkappa, if required.
		auto alphas=Geometry::getCameraFocalLengthPx(Ps.front());
		focal_length_px=std::max(alphas.first,alphas.second);
		// std::cout << "Using CPU inversion.\n";
		std::vector<Eigen::Matrix<float,3,4> > _PinvTs(Ps.size());
		#pragma omp parallel for
		for (int i=0;i<Ps.size();i++)
#ifdef __xprojectionmatrix_hxx
			culaut::projection_matrix_pseudoinverse_transpose(Ps[i].data(),_PinvTs[i].data());
#else
			_PinvTs[i]=Geometry::pseudoInverse(Ps[i]).transpose().cast<float>();
#endif // __xprojectionmatrix_hxx
		PinvTs->download(_PinvTs[0].data(),(int)_PinvTs.size()*3*4);
		std::vector<Eigen::Vector4f> _Cs(Ps.size());
		#pragma omp parallel for
		for (int i=0;i<Ps.size();i++)
#ifdef __xprojectionmatrix_hxx
			culaut::projection_matrix_source_position(Ps[i].data(),_Cs[i].data());
#else
			_Cs[i]=Geometry::getCameraCenter(Ps[i]).cast<float>();
#endif // __xprojectionmatrix_hxx
		Cs->download(_Cs[0].data(),(int)_Cs.size()*4);
		return *this;
	}

	/// Evaluates MetricRadonIntermediate without any transformation of the geometry (out is assumed n*n)
	double MetricRadonIntermediate::evaluate(float * _out)
	{
		int n=(int)Ps.size();
		out->allocate(n*n);
		K01s->allocate(n*(n-1)*8);
		std::vector<float> tmp_data;
		if (!_out) {
			tmp_data = std::vector<float>(n*n, 0.0f);
			_out = &tmp_data[0];
		}

		// Make some space for temporaries
		int n_tmp=use_corr?6:1;
			corr_out->allocate(n_tmp*n*(n-1)/2);
		corr_out->setZero();

		// Download current values of out (so as not to remove values not written to)
		out->download(_out);
		// Compute ECC on the GPU
		epipolarConsistency(
			n_u, n_v, n, *tex_dtrs, 
			n_alpha, n_t, step_alpha, step_t,
			n, *Cs, *PinvTs, 0, 0x0, *K01s, *out,
			(float)getObjectRadius(), (float)dkappa, m_isDerivative, use_corr, *corr_out);

		// Readback intermediate data
		NRRD::Image<float> out_sums(n_tmp,n*(n-1)/2);
		corr_out->readback(out_sums);
		//out_sums.save("out_sums.nrrd");// DEBUG

		// Compute weighted sums of either correlation or SSD
		double weighted_sum=0,sum_over_weights=0;
		int l=out_sums.length();
		if (use_corr)
		{
			#pragma omp parallel for reduction (+:weighted_sum,sum_over_weights)
			for (int ij6=0;ij6<l;ij6+=6) {
				short i,j;
				get_ij(ij6/6,n,i,j);
				float corr=cc(&out_sums[ij6]);
				float weight=out_sums[ij6+5];
				weighted_sum+=(_out[i+j*n]=(1.0f-corr)*weight);
				sum_over_weights+=weight;
			}
		}
		else
		{
			// Read back and sum up
			out->readback(_out);
			//#pragma omp parallel for reduction (+:weighted_sum,sum_over_weights)
			for (int ij=0;ij<l;ij++) {
				short i,j;
				get_ij(ij,n,i,j);
				float weight=out_sums[ij];
				weighted_sum+=(_out[i+j*n]*=weight);
				sum_over_weights+=weight;
			}
		}
		return weighted_sum/sum_over_weights;
	}

	/// Evaluates MetricRadonIntermediate for just specific view
	double MetricRadonIntermediate::evaluate(const std::set<int>& views, float * _out)
	{
		std::vector<Eigen::Vector4i> indices;
		for (auto i = views.begin(); i != views.end(); ++i)
			for (auto j = i; j != views.end(); ++j){
				if (i == j) continue;
				indices.push_back(Eigen::Vector4i(*i, *j, *i, *j));
			}
		// Temporary memory, as needed
		std::vector<float> tmp;
		if (!_out)
		{
			tmp.resize(indices.size());
			_out=&tmp[0];
		}
		// Sum up weights and cost
		return evaluate(indices,_out);
	}

	// Might be useful for debugging: if CUDA crashed, you likely messed up the index array
	bool check_indices(const std::vector<Eigen::Vector4i>& _indices, int n_proj, int n_dtr)
	{
		int n=(int)_indices.size();
		for (int i=0;i<n;i++)
		{
			const int *p2_dtr2=_indices[i].data();
			if (p2_dtr2[0]<0||p2_dtr2[0]>=n_proj)
				return false;
			if (p2_dtr2[1]<0||p2_dtr2[1]>=n_proj)
				return false;
			if (p2_dtr2[2]<0||p2_dtr2[2]>=n_dtr)
				return false;
			if (p2_dtr2[3]<0||p2_dtr2[3]>=n_dtr)
				return false;
		}
		return true;
	}

	/// Evaluates MetricRadonIntermediate for specific pairs and projection geometry. Will write indices.size() float values to out.
	double MetricRadonIntermediate::evaluate(const std::vector<Eigen::Vector4i>& _indices, float * _out)
	{
		#ifdef _DEBUG
			if (!check_indices(_indices,(int)Ps.size(),(int)dtrs.size()))
			{
				std::cerr << "EpipolarConsistency::MetricRadonIntermediate::evaluate(...) runtime error: Index array contains invalid indices.\n";
				return 0.0;
			}
		#endif // _DEBUG

		// Compute ECC for specific indices on the GPU
		int n_pairs=(int)_indices.size();
		indices->download(_indices[0].data(),n_pairs*4);
		out->allocate(n_pairs);
		K01s->allocate(n_pairs*16);
			
		if (use_corr) {
			corr_out->allocate(n_pairs*6);
			corr_out->setZero();
		}
		else  {
			corr_out->allocate(n_pairs);
			corr_out->setZero();
		}

		epipolarConsistency(
			n_u, n_v, (int)dtrs.size(), *tex_dtrs,
			n_alpha, n_t, step_alpha, step_t,
			(int)Ps.size(), *Cs, *PinvTs,
			n_pairs, *indices, *K01s, *out,
			(float)getObjectRadius(), (float)dkappa, m_isDerivative,
			use_corr, *corr_out);

		// Read back intermediate results, if required, compute correlation coefficient
		NRRD::Image<float> out_sums(use_corr?6:1,n_pairs);
		corr_out->readback(out_sums);
		if (use_corr)
		{
			#pragma omp parallel for
			for (int i=0;i<n_pairs;i++)
				_out[i]=1.0f-cc(out_sums+i*6);
		}
		else
			out->readback(_out);

		// Compute weighted sum
		int n_tmp=use_corr?6:1;
		double weighted_sum=0,sum_over_weights=0;
		#pragma omp parallel for reduction (+:weighted_sum,sum_over_weights)
		for (int i=0;i<n_pairs;i++) {
			float weight=out_sums[i*n_tmp+n_tmp-1];
			weighted_sum+=(_out[i]*=weight);
			sum_over_weights+=weight;
		}
		return weighted_sum/sum_over_weights;
	}

	double MetricRadonIntermediate::evaluateForImagePair(int i, int j,
		std::vector<float> *redundant_samples0, std::vector<float> *redundant_samples1,
		std::vector<float> *kappas)
	{
		return evaluateForImagePair(i,j,redundant_samples0,redundant_samples1,kappas,0x0,0x0);
	}

	double MetricRadonIntermediate::evaluateForImagePair(int i, int j,
		std::vector<float> *redundant_samples0, std::vector<float> *redundant_samples1,
		std::vector<float> *kappas, std::vector<std::pair<float,float> > *radon_samples0, std::vector<std::pair<float,float> > *radon_samples1 )
	{
		// Epipolar Geometry
		using namespace Geometry;
		auto& dtr0=*dtrs[i];
		auto& dtr1=*dtrs[j];
		auto P0=Ps[i];
		auto P1=Ps[j];
		auto C0f=getCameraCenter(P0).cast<float>().eval();
		auto C1f=getCameraCenter(P1).cast<float>().eval();
		Eigen::Matrix<float,3,4> P0invTf=pseudoInverse(P0).transpose().cast<float>().eval();
		Eigen::Matrix<float,3,4> P1invTf=pseudoInverse(P1).transpose().cast<float>().eval();
		float K0[8], K1[8];
		computeK01(
			n_u*0.5f,n_v*0.5f,
			C0f.data(),C1f.data(),P0invTf.data(),P1invTf.data(),
			(float)getObjectRadius(),sqrtf((float)(n_u*n_u+n_v*n_v)),(float)dkappa,
			K0,K1);
		// Make sure we have data available on CPU
		dtr0.readback();
		dtr1.readback();
		// Figure out plane angle range and step
		float &dkappa(K1[6]);
		float &kappa_max(K1[7]);
		// Reserve some memory for the results (optional)
		double ecc=0;
		int n=(int)(2*kappa_max/dkappa);
		if (redundant_samples0) redundant_samples0->reserve(n);
		if (redundant_samples1) redundant_samples1->reserve(n);
		if (kappas            ) kappas            ->reserve(n);
		if (radon_samples0    ) radon_samples0    ->reserve(n);
		if (radon_samples1    ) radon_samples1    ->reserve(n);

		// Iterate over plane angle kappa
		for (float kappa=-kappa_max+0.5f*dkappa; kappa<kappa_max; kappa+=dkappa)
		{
			// Compute cosine and sine of kappa
			float x0=cosf(kappa);
			float x1=sinf(kappa);

			// Find corresponding epipolar lines for plane at angle kappa (same as culaut::xgemm<float,3,2,1>(K,x_k,l);)
			float line0[]={K0[0]*x0+K0[3]*x1,K0[1]*x0+K0[4]*x1,K0[2]*x0+K0[5]*x1};
			float line1[]={K1[0]*x0+K1[3]*x1,K1[1]*x0+K1[4]*x1,K1[2]*x0+K1[5]*x1};
	
			// Sample
			float v0=dtr0.sample(line0);
			float v1=dtr1.sample(line1);

			// Store values for plotting, if desired
			if (redundant_samples0) redundant_samples0->push_back(v0);
			if (redundant_samples1) redundant_samples1->push_back(v1);
			if (kappas            ) kappas            ->push_back(kappa);
			if (radon_samples0    ) radon_samples0    ->push_back(std::make_pair(line0[0], line0[1]));
			if (radon_samples1    ) radon_samples1    ->push_back(std::make_pair(line1[0], line1[1]));

			// Accumulate
			ecc=(v0-v1)*(v0-v1)*dkappa;
		}
		
		return ecc;
	}

} // namespace EpipolarConsistency
