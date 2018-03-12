#ifndef __epipolar_consistency_hxx
#define __epipolar_consistency_hxx
// Created by A. Aichert on Thu Nov 21st 2013
// Class to compute Epipolar Consistency given ProjectionMatrices and RadonDerivatives
// This is the header-only CPU version. For CUDA implementation, see EpipolarConsistency.h/cpp/cu

#include <LibProjectiveGeometry/ProjectionMatrix.h>
#include "RadonIntermediate.h"

#include <LibUtilsCuda/culaut/culaut.hxx>

namespace EpipolarConsistency
{
	using Geometry::ProjectionMatrix;
	using Geometry::RP3Point;
	using Geometry::RP3Line;
	using Geometry::RP3Plane;

	/// Simple class to compute a metric for geometric consistency based on the redundancy due to the epipolar constraint and Grangeat's theorem.
	struct MetricCPU {
		std::vector<ProjectionMatrix>		Ps;		//< Copy of projection matrices det(M)>0 and |m_3|=1
		std::vector<RadonIntermediate*>		dtrs;	//< Radon derivatives (same size as Ps)
		double								dkappa; //< Angle between planes

		/// c-tor does not copy image data: you may not delete _dtrs during lifetime.
		MetricCPU(
			std::vector<ProjectionMatrix>&		_Ps,	//< Projection matrices det(M)>0 and |m_3|=1
			std::vector<RadonIntermediate*>	_dtrs,	//< Radon derivatives
			double _dkappa=0.001745329251				//< Angle between planes (defaults to .1 degree) to be removed in later version.
			)
			: Ps(_Ps)
			, dtrs(_dtrs)
			, dkappa(_dkappa)
		{}

		/// Evaluate consistency.
		inline double operator()()
		{
			// Compute camera centers
			std::vector<Eigen::Vector4d> Cs(Ps.size());
			for (int i=0;i<Ps.size();i++)
				Cs[i]=Geometry::getCameraCenter(Ps[i]);
			// Compute metric for all image pairs
			int n=(int)Ps.size();
			double sum=0;
			for (int i=0;i<n;i++)
				#pragma omp parallel for reduction(+:sum)
				for (int r=i+1;r<n;r++)
					sum+=computeForImagePair(Cs[r], Cs[i], Ps[r], Ps[i], dtrs[r], dtrs[i], dkappa);
			return sum/(n*(n-1)*0.5);
		} 

		/// Evaluate consistency for i-th projection.
		inline double operator()(int reference_projection)
		{
			// Quick check for valid params
			int n=(int)Ps.size();
			if (Ps.empty() || dtrs.size()!=(int)n)
				return -1;

			// Compute camera centers
			std::vector<Eigen::Vector4d> Cs(Ps.size());
			for (int i=0;i<Ps.size();i++)
				Cs[i]=Geometry::getCameraCenter(Ps[i]);

			// Compute for all pairs containing reference_projection
			double sum=0;
			int r=reference_projection;
			#pragma omp parallel for reduction(+:sum)
			for (int i=0;i<Ps.size();i++)
				sum+=computeForImagePair(Cs[r], Cs[i], Ps[r], Ps[i], dtrs[r], dtrs[i], dkappa);;
			return sum/(Cs.size()-1);
		}

		/// GPU implementation compiled for CPU execution
		template <typename T>
		static T tex2D(const NRRD::ImageView<T>& tex, float x, float y, float *px_alpha=0x0, float *px_t=0x0)
		{
			double x_px=x*(tex.size(0)-1);
			double y_px=y*(tex.size(1)-1);
			// Pass sampled pixel locations for debugging
			if (px_alpha) *px_alpha=(float)x_px;
			if (px_t) *px_t=(float)y_px;
			return (T)tex(x_px,y_px,0.0);
		}

		static inline float getRedundancy(const float *K, const NRRD::ImageView<float>& dtr, float range_t, float x0, float x1, bool isDerivative,
			float *px_alpha, float *px_t)
		{
			const float Pi=3.14159265359f;

			// Find corresponding epipolar lines for plane at angle kappa (same as culaut::xgemm<float,3,2,1>(K,x_k,l);)
			float line[]={K[0]*x0+K[3]*x1,K[1]*x0+K[4]*x1,K[2]*x0+K[5]*x1};
	
			// Length or normal vector
			float length=sqrt(line[0]*line[0]+line[1]*line[1]);
			// Angle between line normal and x-axis (scaled from 0 to +2)
			line[0]=atan2(line[1],line[0])/Pi+1.0f;
			// Distance to the origin (scaled to DTR bins)
			line[1]=(line[2]/length)/range_t+0.5f;

			// Return zero for lines far outside image
			if (line[1]<0.f||line[1]>1.0f)
				return 0.f;

			// Sample DTR and account for symmetry, up to sign.
			if (line[0]>1)
			{
				if (isDerivative)
					return -tex2D<float>(dtr,2.f-line[0],line[1],px_alpha,px_t);
				else
					return tex2D<float>(dtr,2.f-line[0],line[1],px_alpha,px_t);
			}
			else
				return tex2D<float>(dtr,1.f-line[0],1.f-line[1],px_alpha,px_t);
		}

		static inline void shiftOriginAndNormlaize(float x, float y, float* Ki)
		{
			// Multiplication with inv(H)', where H is a translation by (-x,-y)
			Ki[2]+=x*Ki[0]+y*Ki[1];
			Ki[5]+=x*Ki[3]+y*Ki[4];
			float s0=sqrtf(Ki[0]*Ki[0]+Ki[1]*Ki[1]); // length of normal of l0 (l0 is always finite)
			for (int i=0;i<6;i++) Ki[i]/=s0;
		}

		static inline float computeK01(
			float	n_x2,				//< Half image width. Lines shall be relative to center pixel.
			float	n_y2,				//< Half image height. Lines shall be relative to center pixel.
			float*	const  C0,			//< Source positions 0
			float*	const  C1,			//< Source positions 1
			float*	P0invT,				//< Projection 0 pseudo inverse transpose
			float*	P1invT,				//< Projection 1 pseudo inverse transpose
			float*	K0,					//< out: P0invT*BxdA
			float*	K1					//< out: P1invT*BxdA
			)
		{
			// Compute Plücker coordinates of the baseline
			float B01=C0[0]*C1[1]-C0[1]*C1[0];
			float B02=C0[0]*C1[2]-C0[2]*C1[0];
			float B03=C0[0]*C1[3]-C0[3]*C1[0];
			float B12=C0[1]*C1[2]-C0[2]*C1[1];
			float B13=C0[1]*C1[3]-C0[3]*C1[1];
			float B23=C0[2]*C1[3]-C0[3]*C1[2];
			// Normalize by line moment of baseline
			const float s2=std::sqrt(B12*B12+B02*B02+B01*B01);
			const float s3=std::sqrt(B03*B03+B13*B13+B23*B23);
			// K is a 4x2 matrix mapping [cos(kappa) sin(kappa)] to epipolar plane E_kappa.
			// It constsits of the dual Plücker matrix of the baseline Bxd multiplied with the
			// matrix A=[ [a1; 0], [a2; 0] ]
			const float K[] = {
				+ B12*s3, - B02*s3, + B01*s3, 0,
				- B01*B13 - B02*B23, B01*B03 - B12*B23, B02*B03 + B12*B13, -s2*s2,
			};
			// Multiply by pseudoinverse transposed to map epipolar plane E_kappa to lines l_kappa
			culaut::xgemm<float,3,4,2>(P0invT,K,K0); // K0 is 3x2 and maps [cos(kappa) sin(kappa)] to l0_kappa
			culaut::xgemm<float,3,4,2>(P1invT,K,K1); // K1 is 3x2 and maps [cos(kappa) sin(kappa)] to l1_kappa
			// We want the lines to be relative to the image center, instead of the image corner.
			shiftOriginAndNormlaize(n_x2,n_y2,K0);
			shiftOriginAndNormlaize(n_x2,n_y2,K1);
			return s2/s3;
		}

		static inline void __sincosf(float angle, float *sinf, float *cosf)
		{
			*sinf=std::sin(angle);
			*cosf=std::cos(angle);
		}

		/// Compute geometric consitency metric for one pair of views.
		static double MetricCPU::computeForImagePair(
			const Eigen::Vector4d			&C0,			//< Camera center 0 (dehomogenized)
			const Eigen::Vector4d			&C1,			//< Camera center 1 (dehomogenized)
			const ProjectionMatrix			&P0,			//< Projection matrix for image 0, normalized to det(M)>0 and |m3|=1
			const ProjectionMatrix			&P1,			//< Projection matrix for image 1, normalized to det(M)>0 and |m3|=1
			const RadonIntermediate*		dtr0,			//< Radon derivative of image 0
			const RadonIntermediate*		dtr1,			//< Radon derivative of image 1
			double							dkappa,			//< Angle between planes (to be removed)
			int*							n=0x0,			//< Optional: number of samples for debugging
			std::vector<double>*			samples0=0x0,	//< Optional: plotting and debugging: output of DTR samples
			std::vector<double>*			samples1=0x0,	//< Optional: plotting and debugging: output of DTR samples
			std::vector<double>*			angles=0x0,		//< Optional: plotting and debugging: angles for samples
			std::vector<Eigen::Vector4d> *	dtr_samples=0x0 //< Pixel locations sampled in dtr
			)
		{
			using namespace Geometry;
			std::vector<double>	s0;
			std::vector<double>	s1;
			std::vector<double> a;

			float Pi=(float)Geometry::Pi;

			bool debug_out=samples0&&samples1&&angles;
			if (debug_out) *samples0=*samples1=*angles=std::vector<double>();

			// If camera centers are too close to one another, the solution becomes unstable
			if ((C1-C0).norm()<1e-12) return 0;

			///////////////////////////////////////////////////////////////////////////////////////////////
			// Stuff that is pre-computed for GPU version
			float n_x2=dtr0->getOriginalImageSize(0)*0.5f;
			float n_y2=dtr0->getOriginalImageSize(1)*0.5f;
			Eigen::Vector4f C0f=C0.cast<float>();
			Eigen::Vector4f C1f=C1.cast<float>();
			float step_alpha=(float)dtr0->getStepRadonBinning(0);
			float step_t=(float)dtr0->getStepRadonBinning(1);
			float n_t=(float)dtr0->data().size(0);
			float n_alpha=(float)dtr0->data().size(1);
			Eigen::Matrix<float,3,4> P0pT_e=Geometry::pseudoInverse(P0).transpose().cast<float>();
			Eigen::Matrix<float,3,4> P1pT_e=Geometry::pseudoInverse(P1).transpose().cast<float>();
			float *P0invT=P0pT_e.data();
			float *P1invT=P1pT_e.data();
			float range_t=n_t*step_t;
			bool isDerivative=dtr0->isDerivative();
			// end precomputed stuff
			///////////////////////////////////////////////////////////////////////////////////////////////

			// kernelEpipolarConsistencyComputeK01
			float K0[6];
			float K1[6];
			computeK01(
				n_x2, n_y2,
				C0f.data(),
				C1f.data(),
				P0invT,
				P1invT,
				K0,
				K1
			);

			// kernelEpipolarCosistency
			float sum=0;
			// The loop essentially contains consistencyForPlusMinusKappa function
			int nsamples=0;
			for (float kappa=(float)(0.5*dkappa);kappa<Pi;kappa+=(float)dkappa)
			{
		
				// Get point on unit circle of two-space for projection to epipolar lines
				float x_kappa[2];
				__sincosf(kappa,x_kappa+1,x_kappa);

				// Compare redundant information for +kappa
				float px_alpha0,px_t0,
					  px_alpha1,px_t1;
				float vp0=getRedundancy(K0,dtr0->data(),range_t,x_kappa[0],x_kappa[1],isDerivative,&px_alpha0,&px_t0);
				float vp1=getRedundancy(K1,dtr1->data(),range_t,x_kappa[0],x_kappa[1],isDerivative,&px_alpha1,&px_t1);
				if (dtr_samples) dtr_samples->push_back(Eigen::Vector4d(px_alpha0,px_t0,px_alpha1,px_t1));

				// Compare redundant information for -kappa
				x_kappa[0]*=-1;
				float vm0=getRedundancy(K0,dtr0->data(),range_t,x_kappa[0],x_kappa[1],isDerivative,&px_alpha0,&px_t0);
				float vm1=getRedundancy(K1,dtr1->data(),range_t,x_kappa[0],x_kappa[1],isDerivative,&px_alpha1,&px_t1);
				if (dtr_samples) dtr_samples->push_back(Eigen::Vector4d(px_alpha0,px_t0,px_alpha1,px_t1));

				// Return consistency
				float consistency=(vp0-vp1)*(vp0-vp1)+(vm0-vm1)*(vm0-vm1);
				sum+=(float)(consistency*dkappa);
				
				nsamples+=2;

				// For plotting 1D redundant signals
				if (debug_out)
				{
					samples0->push_back(vp0);
					samples1->push_back(vp1);
					angles->push_back(kappa/Pi*180.);
					samples0->push_back(isDerivative?-vm0:vm0);
					samples1->push_back(isDerivative?-vm1:vm1);
					angles->push_back(-kappa/Pi*180.);
				}

				if (vp0==0 && vp1==0 && vm0==0 && vm1==0 )
					break;
			}

			if (n) *n=nsamples;

			// Sort values by angle
			if (debug_out)
			{
				int n=(int)samples0->size();
				std::map<double,std::pair<double,double>> plot;
				for (int i=0;i<n;i++)
					plot[(*angles)[i]]=std::pair<double,double>((*samples0)[i],(*samples1)[i]);
				int i=0;
				for (auto it=plot.begin();it!=plot.end();++it)
				{
					(*samples0)[i]=it->second.first;
					(*samples1)[i]=it->second.second;
					(*angles)[i]=it->first;
					i++;
				}
			}

			return sum;
		}

	};

} // namespace EpipolarConsistency
		

#endif // __epipolar_consistency_hxx
