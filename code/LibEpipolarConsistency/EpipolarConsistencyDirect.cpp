#include "EpipolarConsistencyDirect.h"

// Simple CUDA Wrappers
#include <LibUtilsCuda/CudaMemory.h>
#include <LibUtilsCuda/CudaBindlessTexture.h>

// Transforming line coordinates
#include <LibEpipolarConsistency/RectifiedFBCC.h>

/// Actual work done in CUDA
extern void cuda_computeLineIntegrals(
	short n_lines,                               // Number of lines
	float* lines_d, short line_stride,           // Lines in Hessian normal form and number of float values to next line 
	float *fbcc_d, short fbcc_stride,            // Optional: FBCC_weighting_info for rectification and source-distance-weighting.
	cudaTextureObject_t I, short n_u, short n_v, // The image and its size
	float *integrals_out_d);                     // Output memory: the integrals (size is n_lines)

namespace EpipolarConsistency
{

	/// Utility function: Compute epipolar lines for all angles given in vector kappas.
	std::vector<std::pair<Eigen::Vector3f,Eigen::Vector3f> > computeEpipolarLines(
		const ProjectionMatrix& P0, const ProjectionMatrix& P1,
		const std::vector<float>& kappas)
	{
		using namespace Geometry;

		// Plücker coordinates of the baseline.
		auto C0=getCameraCenter(P0);
		auto C1=getCameraCenter(P1);
		auto B =join_pluecker(C0,C1);

		// Projection of planes to lines in the image.
		auto P0invT=pseudoInverse(P0).transpose().eval();
		auto P1invT=pseudoInverse(P1).transpose().eval();
		// Figure out epipolar planes at 0 and 90 degrees w.r.t. the origin.
		RP3Plane E0 =join_pluecker(B,origin3);
		RP3Plane E90=join_pluecker(B,E0);
		// Convert to Hessian normal form
		E0 /=E0 .head(3).norm();
		E90/=E90.head(3).norm();

		// Loop over all kappas
		int n=(int)kappas.size();
		std::vector<std::pair<Eigen::Vector3f,Eigen::Vector3f> > corresponding_epipolar_lines(n);
	#pragma omp parallel for
		for (int i=0;i<n;i++)
		{
			const double& kappa=kappas[i];
			// Epipolar plane at angle kappa to the origin and epipolar line on reference image.
			RP3Plane  E_kappa=cos(kappa)*E0+sin(kappa)*E90;
			// Projection from plane to line via pseudoinverse-transpose.
			RP2Line  l0_kappa=P0invT*E_kappa;
			RP2Line  l1_kappa=P1invT*E_kappa;
			// Scale to Hessian normal form.
			l0_kappa/=l0_kappa.head(2).norm();
			l1_kappa/=l1_kappa.head(2).norm();
			// Return pairs of corresponding epipolar lines
			corresponding_epipolar_lines[i]=std::make_pair(l0_kappa.cast<float>(),l1_kappa.cast<float>());
		}
		return corresponding_epipolar_lines;
	}

	double computeForImagePair(
		const Geometry::ProjectionMatrix&          P0, const Geometry::ProjectionMatrix&          P1,
		const UtilsCuda::BindlessTexture2D<float>& I0, const UtilsCuda::BindlessTexture2D<float>& I1,
		double dkappa, double object_radius_mm, bool fbcc,
		std::vector<float> *redundant_samples0, std::vector<float> *redundant_samples1,
		std::vector<float> *kappas)
	{

		using namespace Geometry;

		// Use temporaries if optional output argumemts are not provided
		std::vector<float> tmp0,tmp1,tmp2;
		if (!redundant_samples0) redundant_samples0=&tmp0;
		if (!redundant_samples1) redundant_samples1=&tmp1;
		if (!kappas)             kappas            =&tmp2;

		// Pixel spacing on virtual detector plane (not actually relevant, just here for clarity of code)
		double pixel_spacing=1.0;

		// If not provided, estimate object radius via field of view
		if (object_radius_mm<=0) {
			object_radius_mm=std::max(estimateObjectRadius(P0,I0.size[0],I0.size[1]),estimateObjectRadius(P1,I1.size[0],I1.size[1]));
		}

		// Figure out which epipolar planes are relevant
		auto C0=getCameraCenter(P0);
		auto C1=getCameraCenter(P1);
		auto B =join_pluecker(C0,C1);
		auto range_kappa=estimateAngularRange(B,object_radius_mm);

		// If not provided, figure out appropriate dkappa (so that we draw as many samples as there are pixels on the diagonal)
		if (dkappa<=0)
		{
			double diag=std::sqrt(I0.size[0]*I0.size[0]+I0.size[1]*I0.size[1]);
			dkappa=0.5*(range_kappa.second-range_kappa.first)/diag;
		}

		// Compute kappas (unless provided)
		int n_lines=(int)kappas->size();
		if (kappas->empty())
		{
			n_lines=(int)((range_kappa.second-range_kappa.first)/dkappa);
			kappas->resize(n_lines);
			#pragma omp parallel for
				for (int i=0;i<n_lines;i++)
					(*kappas)[i]=(float)(range_kappa.first+dkappa*i);
		}

		// Compute epipolar lines and download to GPU
		auto lines01=computeEpipolarLines(P0,P1,*kappas);
		UtilsCuda::MemoryBlock<float> l01s_d(n_lines*6,lines01.front().first.data());

		// Allocate space for results
		UtilsCuda::MemoryBlock<float> v0s_d(n_lines);
		UtilsCuda::MemoryBlock<float> v1s_d(n_lines);

		// Compute integrals in image 0 and 1
		if (!fbcc)
		{
			// ECC
			cuda_computeLineIntegrals(n_lines, (float*)l01s_d  , 6, 0x0, 0, I0, I0.size[0], I0.size[1], v0s_d);
			cuda_computeLineIntegrals(n_lines, (float*)l01s_d+3, 6, 0x0, 0, I1, I1.size[0], I1.size[1], v1s_d);
		}
		else
		{
			// Virtual detector plane
			auto d =pluecker_direction(B);
			auto m =pluecker_moment(B);
			Eigen::Vector3d		U=d.normalized();
			Eigen::Vector3d		V=m.normalized();
			Geometry::RP3Plane	E(0,0,0,0);
			E.head(3)=U.cross(V);

			// Orthogonal projection to E
			Geometry::ProjectionMatrix P_E=Geometry::ProjectionMatrix::Zero();
			P_E.block<1,3>(0,0)=U.transpose();
			P_E.block<1,3>(1,0)=V.transpose();
			P_E(2,3)=pixel_spacing;
			// Rectifying homographies
			Geometry::RP2Homography H0=P_E*centralProjectionToPlane(C0,E)*pseudoInverse(P0);
			Geometry::RP2Homography H1=P_E*centralProjectionToPlane(C1,E)*pseudoInverse(P1);

			// Figure out how line coordinates transform under H0/1
			std::vector<FBCC_weighting_info> fbcc01(2*n_lines);
			for (int i=0;i<n_lines;i++) {
				// Corresponding epipolar lines.
				const auto& l0_kappa=lines01[i].first ;
				const auto& l1_kappa=lines01[i].second;
				// Epipoar plane
				RP3Plane E_kappa=P0.transpose()*l0_kappa.cast<double>();

				// Plane orthogonal to baseline through C0/1
				RP3Plane EB0(d(0),d(1),d(2),-d.dot(C0.head(3)));
				RP3Plane EB1(d(0),d(1),d(2),-d.dot(C1.head(3)));
				// Compute intersection of detector, epipolar plane and plane orthogonal to baseline through C0/1
				RP3Point Ak0=Geometry::meet_pluecker(Geometry::meet_pluecker(EB0,E_kappa),E);
				RP3Point Ak1=Geometry::meet_pluecker(Geometry::meet_pluecker(EB1,E_kappa),E);
				Geometry::dehomogenize(Ak0);
				Geometry::dehomogenize(Ak1);
				// Distance of C0/1 to 3D epipolar line in pixels
				float d_l_kappa_C0_px=(float)((Ak0-C0).norm()/pixel_spacing);
				float d_l_kappa_C1_px=(float)((Ak1-C1).norm()/pixel_spacing);
				// Projection of Ak to image 0/1
				RP2Point ak0=P0*Ak0;
				RP2Point ak1=P1*Ak1;
				Geometry::dehomogenize(ak0);
				Geometry::dehomogenize(ak1);

				// Transformation of line coordinates on l0_kappa and l1_kappa to virtual detector u-axis.
				LinePerspectivity phi0(H0, l0_kappa.cast<double>());
				LinePerspectivity phi1(H1, l1_kappa.cast<double>());

				// Line coordintas of ak0/1 in virtual detector
				float t_prime_ak0=phi0.transform((float)phi0.project_to_line(ak0,l0_kappa.cast<double>()));
				float t_prime_ak1=phi1.transform((float)phi1.project_to_line(ak1,l1_kappa.cast<double>()));

				// Store in float memory block for GPU download
				fbcc01[2*i+0].phi            =phi0;
				fbcc01[2*i+1].phi            =phi1;
				fbcc01[2*i+0].t_prime_ak     =t_prime_ak0;
				fbcc01[2*i+1].t_prime_ak     =t_prime_ak1;
				fbcc01[2*i+0].d_l_kappa_C_sq =d_l_kappa_C0_px*d_l_kappa_C0_px;
				fbcc01[2*i+1].d_l_kappa_C_sq =d_l_kappa_C1_px*d_l_kappa_C1_px;
			}

			// Download phis to GPU
			short n_fbcc=sizeof(FBCC_weighting_info)/sizeof(float);
			UtilsCuda::MemoryBlock<float> fbcc01_d(2*n_lines*n_fbcc,(float*)fbcc01.data());
			
			// FBCC
			cuda_computeLineIntegrals(n_lines, (float*)l01s_d  , 6, (float*)fbcc01_d        , n_fbcc*2, I0, I0.size[0], I0.size[1], v0s_d);
			cuda_computeLineIntegrals(n_lines, (float*)l01s_d+3, 6, (float*)fbcc01_d+n_fbcc , n_fbcc*2, I1, I1.size[0], I1.size[1], v1s_d);
		}
		
		// Read back
		auto &v0s(*redundant_samples0), &v1s(*redundant_samples1);
		v0s.resize(n_lines*3);
		v0s_d.readback(v0s.data());
		v1s.resize(n_lines*3);
		v1s_d.readback(v1s.data());

		// Return sum of squared differences of redundant samples
		double metric=0;
		for (int i=0;i<n_lines;i++)
			metric+=(v0s[i]-v1s[i])*(v0s[i]-v1s[i])*dkappa;
		return metric;
	}


	MetricDirect::MetricDirect(const std::vector<ProjectionMatrix>& _Ps, const std::vector<UtilsCuda::BindlessTexture2D<float>*>& _Is) {
		use_fbcc=false;
		setProjectionMatrices(_Ps);
		setProjectionImages(_Is);
	}

	Metric& MetricDirect::setProjectionMatrices(const std::vector<ProjectionMatrix>& _Ps)
	{
		Ps=_Ps;
		return *this;
	}

	Metric& MetricDirect::setProjectionImages(const std::vector<UtilsCuda::BindlessTexture2D<float>*>& _Is)
	{
		if (!_Is.empty())
		{
			n_u=_Is.front()->size[0];
			n_v=_Is.front()->size[1];
		}
		Is=_Is;
		return *this;
	}

	int  MetricDirect::getNumberOfProjetions()
	{
		return (int)Is.size();
	}

	double MetricDirect::evaluate(float *out)
	{
		double cost=0;
		int n=getNumberOfProjetions();
		NRRD::ImageView<float> cost_image(n,n,1,out);
		for (int i=0;i<n;i++)
			for (int j=i+1;j<n;j++) {
				double ecc=evaluateForImagePair(i,j);
				cost+=ecc;
				if (out) cost_image.pixel(i,j)=(float)ecc;
			}
		return cost;
	}

	double MetricDirect::evaluateForImagePair(int i, int j,
		std::vector<float> *redundant_samples0, std::vector<float> *redundant_samples1,
		std::vector<float> *kappas)
	{
		return computeForImagePair(
			Ps[i],Ps[j], *Is[i],*Is[j],
			dkappa, getObjectRadius(), use_fbcc,
			redundant_samples0, redundant_samples1,
			kappas
			);
	}

	MetricDirect& MetricDirect::setFanBeamConsistency(bool fbcc){
		use_fbcc=fbcc;
		return *this;
	}

} // namespace EpipolarConsistency
