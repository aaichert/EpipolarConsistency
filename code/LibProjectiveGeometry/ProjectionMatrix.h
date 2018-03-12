#ifndef __projection_matrix
#define __projection_matrix
// Created by A. Aichert on Mon Nov 18th 2013
// Useful linear algebra for the pinhole camera model and epipolar geometry.

#include "ProjectiveGeometry.hxx"

namespace Geometry
{
	typedef Eigen::Matrix<double,4,3> ProjectionMatrixInverse;
	typedef Eigen::Matrix<double,3,3> FundamentalMatrix;
	
	/// Normalize a camera matrix P=[M|p4] by -sign(det(M))/||m3|| such that (du,dv,d)'=P(X,Y,Z,1)' encodes the depth d
	void normalizeProjectionMatrix(ProjectionMatrix& P);

	/// Backprojection
	ProjectionMatrixInverse pseudoInverse(const ProjectionMatrix&);

	/// Decompose Projection Matrix into K[R|t] using RQ-decomposition. Returns false if R is left-handed (For RHS world coordinate systems, this implies imageVPointsUp is wrong).
	bool projectionMatrixDecomposition(const ProjectionMatrix& P, Eigen::Matrix3d& K, Eigen::Matrix3d& R, Eigen::Vector3d& t, bool imageVPointsUp=true);
	/// Decompose projection matrix and return intrinsic parameters in upper triangular 3x3 matrix
	Eigen::Matrix3d getCameraIntrinsics(const ProjectionMatrix& P);
	
	/// Extract the world coordinates of the camera center from a projection matrix.
	RP3Point getCameraCenter(const ProjectionMatrix& P);

	/// Compute the principal point via M*m3.
	RP2Point getCameraPrincipalPoint(const ProjectionMatrix& P);

	/// Direction of principal ray from a projection matrix. (Normal to principal plane, which is last row of P)
	Eigen::Vector3d getCameraDirection(const ProjectionMatrix& P);

	/// Compute the two three-points where the image u- and v-axes meet infinity.
	std::pair<RP3Point,RP3Point> getCameraAxisDirections(const ProjectionMatrix& P);

	/// Compute the focal length in pixels (diagonal entries of K in P=K[R t] ). Assumes normalized projection matrix.
	std::pair<double, double> getCameraFocalLengthPx(const ProjectionMatrix& P);

	/// Decomposes the projection matrix to compute the equation of the image plane. For left-handed coordinate systems, pixel_spacing can be negated.
	RP3Plane getCameraImagePlane(const ProjectionMatrix& P, double pixel_spacing);

	/// Intrinsic parameters of a pinhole camera as a 3x3 matrix
	inline Eigen::Matrix3d makeCalibrationMatrix(double ax, double ay, double u0, double v0, double skew=0.0)
	{
		Eigen::Matrix3d K=Eigen::Matrix3d::Identity();
		K(0,0)=ax;
		K(1,1)=ay;
		K(0,2)=u0;
		K(1,2)=v0;
		K(0,1)=skew;
		return K;
	}

	/// Compose a projection matrix from rotation matrix, translation vector and intrinsic parameters
	ProjectionMatrix makeProjectionMatrix(
		const Eigen::Matrix3d& K,
		const Eigen::Matrix3d& R=Eigen::Matrix3d::Identity(),
		const Eigen::Vector3d& t=Eigen::Vector3d(0,0,0)
		);

	/// Compute fundamental matrix from two projection matrices. Pseudoinverse-based implementation
	FundamentalMatrix computeFundamentalMatrix(const ProjectionMatrix& P0, const ProjectionMatrix& P1);

} // namespace Geometry

#endif // __projection_matrix
