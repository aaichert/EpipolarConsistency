
#include "ProjectionMatrix.h"

#include "SingularValueDecomposition.h"

#include <Eigen/Dense>

namespace Geometry
{

	/// Normalize a camera matrix P=[M|p4] by -sign(det(M))/||m3|| such that (du,dv,d)'=P(X,Y,Z,1)' encodes the depth d
	void normalizeProjectionMatrix(ProjectionMatrix& P)
	{
		double norm_m3=P.block<1,3>(2,0).norm();
		double detM=P.block<3,3>(0,0).determinant();
		if (detM<0) norm_m3=-norm_m3;
		P=P*(1.0/norm_m3);
	}

	/// Backprojection
	ProjectionMatrixInverse pseudoInverse(const ProjectionMatrix& P)
	{
		return pseudoInverse(Eigen::MatrixXd(P));
	}

	/// Decompose Projection Matrix into K[R|t] using RQ-decomposition. Returns false if R is left-handed (For RHS world coordinate systems, this implies imageVPointsUp is wrong).
	bool projectionMatrixDecomposition(const ProjectionMatrix& P, Eigen::Matrix3d& K, Eigen::Matrix3d& R, Eigen::Vector3d& t, bool imageVPointsUp)
	{
		// Compute RQ decomposition of leftmost 3x3 sub-matrix of P
		// Uses a permutation of rows and the QR decomposition of the transpose.
		Eigen::Matrix3d M=P.block<3,3>(0,0);
		M.row(0).swap(M.row(2));
		M.transposeInPlace();
		// Compute PA'=QR
		auto QR=M.householderQr();
		// Permute to produce A=(P'RB)B'Q = R*Q* 
		R=QR.householderQ().transpose();
		K=(R*M).transpose();
		R.row(0).swap(R.row(2));
		K.row(0).swap(K.row(2));
		K.col(0).swap(K.col(2));
		// make diagonal of K positive
		Eigen::Matrix3d S;
		S.setIdentity();
		if (K(0,0)<0) S(0,0)=-1;
		if (K(1,1)<0) S(1,1)=-1;
		if (imageVPointsUp) S(1,1)*=-1;
		if (K(2,2)<0) S(2,2)=-1;
		K=K*S;
		R=S*R;
		// Force zero elements in K
		K(1,0)=K(2,0)=K(2,1)=0;
		// Scale
		K*=1.0/K(2,2);
		t=K.lu().solve(P.block<3,1>(0,3));
		return R.determinant()>0;
	}


	/// Decompose projection matrix and return intrinsic parameters in upper triangular 3x3 matrix
	Eigen::Matrix3d getCameraIntrinsics(const ProjectionMatrix& P)
	{
		Eigen::Matrix3d K, R;
		Eigen::Vector3d t;
		projectionMatrixDecomposition(P, K, R, t);
		return K;
	}

	/// Extract the world coordinates of the camera center from a projection matrix. (SVD based implementation)
	RP3Point getCameraCenter(const ProjectionMatrix& P)
	{
		Eigen::Vector4d C=nullspace(P);
		if (C(3)<-1e-12 || C(3)>1e-12)
			C=C/C(3); // Def:Camera centers are always positive.
		return C;
	}

	/// Compute the principal point via M*m3.
	RP2Point getCameraPrincipalPoint(const ProjectionMatrix& P)
	{
		auto pp=P.block<3,3>(0,0)*P.block<1,3>(2,0).transpose();
		return pp/pp(2);
	}

	/// Direction of principal ray from a projection matrix. (Normal to principal plane, which is last row of P)
	Eigen::Vector3d getCameraDirection(const ProjectionMatrix& P)
	{
		Eigen::Vector3d m3=P.block<1,3>(2,0);
		if (m3.norm()>1e-12)
			m3.normalize();
		return m3;
	}

	/// Compute the two three-points where the image u- and v-axes meet infinity.
	std::pair<RP3Point,RP3Point> getCameraAxisDirections(const ProjectionMatrix& P)
	{
		Eigen::Vector3d m1=P.block<1,3>(0,0);
		Eigen::Vector3d m2=P.block<1,3>(1,0);
		Eigen::Vector3d m3=P.block<1,3>(2,0);
		return std::make_pair(infinitePoint(m3.cross(m2)), infinitePoint(m3.cross(m1)));
	}

	/// Compute the focal length in pixels (diagonal entries of K in P=K[R t] ). Assumes normalized projection matrix.
	std::pair<double, double> getCameraFocalLengthPx(const ProjectionMatrix& P)
	{
		Eigen::Vector3d m1=P.block<1,3>(0,0);
		Eigen::Vector3d m2=P.block<1,3>(1,0);
		Eigen::Vector3d m3=P.block<1,3>(2,0);
		Eigen::Vector3d U=m3.cross(m2).normalized();
		Eigen::Vector3d V=m3.cross(m1).normalized();
		return std::make_pair(m1.dot(V.cross(m3)), m2.dot(U.cross(m3)) );
		// TODO Is this always identical?
		// Compute focal length via <m1,m1>-pp² resp. <m2,m2>-pp²
		// sqrt(P(1,1:3)*P(1,1:3)' - K(1,3)^2);

	}

	/// Decomposes the projection matrix to compute the equation of the image plane. For left-handed coordinate systems, pixel_spacing can be negated.
	RP3Plane getCameraImagePlane(const ProjectionMatrix& P, double pixel_spacing)
	{
		// Compute focal length in world units (typically millimeters)
		Eigen::Matrix3d K, R;
		Eigen::Vector3d t;
		projectionMatrixDecomposition(P, K, R, t);
		double focalLengthX=K(0,0)*pixel_spacing;
		// Compute image plane by moving the principal plane back by focal length
		RP3Plane principal_plane=P.block<1,4>(2,0);
		principal_plane=principal_plane/principal_plane.head(3).norm();
		return RP3Plane(principal_plane[0],principal_plane[1],principal_plane[2],principal_plane[3]-focalLengthX);
	}

	/// Compose a projection matrix from rotation matrix, translation vector and intrinsic parameters
	ProjectionMatrix makeProjectionMatrix(
		const Eigen::Matrix3d& K,
		const Eigen::Matrix3d& R,
		const Eigen::Vector3d& t
		)
	{
		ProjectionMatrix P;
		P.block<3,1>(0,3)=K*t;
		Eigen::Matrix3d M=K*R;
		P.block<3,3>(0,0)=M;
		normalizeProjectionMatrix(P);
		return P;
	}

	/// Compute fundamental matrix from two projection matrices. Pseudoinverse-based implementation
	FundamentalMatrix computeFundamentalMatrix(const ProjectionMatrix& P0, const ProjectionMatrix& P1)
	{
		// Compute epipole as projection of 'other' camera center
		Eigen::Vector4d C0=getCameraCenter(P0);
		Eigen::Vector3d e1=P1*C0;
		// Create skew-symmetric matrix for e1
		Eigen::Matrix3d e1x;
		e1x <<   0, +e1[2],  -e1[1],
			-e1[2],      0, +e1[0],
			+e1[1], -e1[0],      0;
		// Compute pseudo inverse
		ProjectionMatrixInverse P0plus=pseudoInverse(P0);
		// Compute F=e1x*P'P+
		FundamentalMatrix F=e1x*P1*P0plus;
		return F;
	}

} // namespace Geometry
