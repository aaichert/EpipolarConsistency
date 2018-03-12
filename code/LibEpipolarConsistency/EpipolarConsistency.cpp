#include "EpipolarConsistency.h"

#include <Eigen/Dense>

namespace EpipolarConsistency
{
	using Geometry::Pi;

	/// 2DO Should use SVD from Geometry module to reduce compile time.
	Geometry::RP3Point estimateIsoCenter(const std::vector<ProjectionMatrix> &Ps)
	{
		int n=(int)Ps.size();
		Eigen::Matrix3d A=Eigen::Vector3d(n,n,n).asDiagonal();
		Eigen::Vector3d b(0,0,0);
		// Distance as norm of a point projected to plane orthogonal to view direction through the origin.
		for (int i=0;i<n;i++)
		{
			const ProjectionMatrix &P(Ps[i]);
			// Center of projection
			Eigen::Vector3d C=Geometry::getCameraCenter(P).block<3,1>(0,0);
			// View direction
			Eigen::Vector3d V=P.block<1,3>(2,0).normalized();
			// O=id-v*v^T is projection in direction v.
			A-=V*V.transpose();
			// For points x on the principal ray it holds that O*x=O*C
			b+=C-V*(V.transpose()*C);
		}
		// Solve by SVD
		Eigen::JacobiSVD<Eigen::Matrix3d> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Geometry::RP3Point O(0,0,0,1);
		O.block<3,1>(0,0)=svd.solve(b);
		return O;
	}

	double estimateObjectRadius(const ProjectionMatrix& P, int n_u, int n_v)
	{
		using namespace Geometry;
		using namespace std;
		// Get the focal lengths in pixels.
		auto   alphas=getCameraFocalLengthPx(P);
		// Compute field of view (use maximum of u and v).
		double fov   =max(abs(atan(0.5*n_u/alphas.first)), abs(atan(0.5*n_v/alphas.second)));
		// The source to iso-center distance.
		double sid   =getCameraCenter(P).head(3).norm();
		// Return approximate radius of an circumscribed  sphere.
		return sin(fov)*sid;
	}

	std::pair<double,double> estimateAngularRange(const RP3Line& B, double object_radius_mm)
	{
		// Distance of baseline to origin
		double baseline_dist=Geometry::pluecker_distance_to_origin(B);
		// If the baseline intersects the object, ECC is not well-defined. We return a half circle anyway.
		if (baseline_dist<=object_radius_mm)
			return std::make_pair(-0.5*Pi,0.5*Pi);
		// Else, find angle of plane which touches the sphere
		double kappa_max=std::abs(std::asin(object_radius_mm/baseline_dist));
		return std::make_pair(-kappa_max,kappa_max);
	}

	double estimateAngularStep(const ProjectionMatrix& P0, const ProjectionMatrix& P1, int n_u, int n_v)
	{
		using namespace Geometry;
		double radius_mm=std::max(estimateObjectRadius(P0,n_u,n_v),estimateObjectRadius(P1,n_u,n_v));
		auto baseline=join_pluecker(getCameraCenter(P0),getCameraCenter(P1));
		auto range_kappa=estimateAngularRange(baseline,radius_mm);
		return (range_kappa.second-range_kappa.first)/std::sqrt(n_u*n_u+n_v*n_v);
	}

	Metric& Metric::setObjectRadius(double radius_mm)
	{
		object_radius_mm=radius_mm;
		return *this;
	}

	double Metric::getObjectRadius() const
	{
		if (object_radius_mm>0)
			return object_radius_mm;
		if (getProjectionMatrices().empty())
			return 0;
		auto P=getProjectionMatrices().front();
		return estimateObjectRadius(P,n_u,n_v);
	}

	Metric& Metric::setEpipolarPlaneStep(double dkappa_rad) {
		dkappa=dkappa_rad;
		return *this;
	}

	Metric& Metric::setProjectionMatrices(const std::vector<ProjectionMatrix>& _Ps)
	{
		Ps=_Ps;
		return *this;
	}

	const std::vector<ProjectionMatrix>&Metric:: getProjectionMatrices() const
	{
		return Ps;
	}

	Metric::Metric() : object_radius_mm(0), dkappa(0), n_u(0), n_v(0) {}

	Metric::~Metric() {}

} // namespace EpipolarConsistency
