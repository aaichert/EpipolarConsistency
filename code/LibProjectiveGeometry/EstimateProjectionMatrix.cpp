#include "EstimateProjectionMatrix.h"

#include "ProjectiveGeometry.hxx"
#include "ProjectionMatrix.h"
#include "SingularValueDecomposition.h"


namespace Geometry {

	/// Normalization of 2D and 3D point clouds: de-mean and scale to +/- sqtr(2), sqrt(3) respectively. Input assumed to have homogeneous coordinate 1.
	std::pair<Geometry::RP2Homography,Geometry::RP3Homography> dlt_normalization(
		const std::vector<Eigen::Vector3d>& p2d, 
		const std::vector<Eigen::Vector4d>& p3d,
		const std::map<int,int>& match)
	{
		// Compute mean
		Eigen::Vector2d mean2d(0, 0);
		Eigen::Vector3d mean3d(0, 0, 0);
		for (auto it=match.begin();it!=match.end();++it) {
			mean2d += p2d[it->first ].head(2);
			mean3d += p3d[it->second].head(3);
		}
		mean2d /= (double)match.size();
		mean3d /= (double)match.size();

		// Compute size
		double s2d = 0;
		double s3d = 0;
		for (auto it=match.begin();it!=match.end();++it) {
			s2d += (p2d[it->first ].head(2) - mean2d).norm();
			s3d += (p3d[it->second].head(3) - mean3d).norm();
		}
		s2d *= sqrt(2) /(double)match.size();
		s3d *= sqrt(3) /(double)match.size();

		// Compose normalization matrices
		return std::make_pair(
				Geometry::Scale(s2d,s2d    )*Geometry::Translation(-mean2d[0],-mean2d[1]),
				Geometry::Scale(s3d,s3d,s3d)*Geometry::Translation(-mean3d)
			);
	}

	/// Direct Linear Transformation for projection matrices. Input assumed to have homogeneous coordinate 1.
	Geometry::ProjectionMatrix dlt(const std::vector<Eigen::Vector3d>& x, const std::vector<Eigen::Vector4d>& X, std::map<int,int> match)
	{
		// Optional match parameter
		if (match.empty() && X.size()==x.size() ) {
			for (int i=0;i<(int)X.size();i++)
				match[i]=i;
		}
	
		// Check for insufficiant data
		if (match.size() < 6 ) {
			return Geometry::ProjectionMatrix::Zero();
		}

		// Normalization of input data
		auto  normalization=dlt_normalization(x,X,match);
		//auto& N_px=normalization.first;
		//auto& N_mm=normalization.second;

		auto& N_px=Geometry::RP2Homography::Identity();
		auto& N_mm=Geometry::RP3Homography::Identity();

		// Build homgeneous system mmatrix from point matches
		Eigen::MatrixXd A(2 * match.size(), 12);
		auto it=match.begin();
		for (int k = 0; it!=match.end(); k+=2, ++it)
		{
			// Normalize input points
			Geometry::RP2Point  x_norm = N_px*x[it->first];
			Geometry::RP3Point  X_norm = N_mm*X[it->second];
			// Write two rows in A (we get two independent equations from one point match)
			A.block<1, 4>(k    , 0).setZero();
			A.block<1, 4>(k + 1, 4).setZero();
			A.block<1, 4>(k    , 4) =  x_norm(2) *X_norm.transpose();
			A.block<1, 4>(k + 1, 0) = -x_norm(2) *X_norm.transpose();
			A.block<1, 4>(k    , 8) = -x_norm(1) *X_norm.transpose();
			A.block<1, 4>(k + 1, 8) =  x_norm(0) *X_norm.transpose();
		}

		// Solve and reshape
		Eigen::VectorXd p = Geometry::nullspace(A);
		Geometry::ProjectionMatrix P_norm = Eigen::Matrix<double, 4, 3>(p.data()).transpose();

		//denormalize
		Geometry::ProjectionMatrix P = N_px.inverse()*P_norm*N_mm;
		Geometry::normalizeProjectionMatrix(P);
		return P;
	}

} // namespace Geometry

