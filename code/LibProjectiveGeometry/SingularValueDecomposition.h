#ifndef __singular_value_decomposition_h
#define __singular_value_decomposition_h
// Created by A. Aichert on Mon Nov 18th 2013
// Eigen's SVD is compiled seperately to reduce overall compilation time.

#include <Eigen/Dense>

namespace Geometry
{
	/// Compute the pseudo-inverse of a matrix
	Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd& A);

	/// Compute the pseudo-inverse and null-space of a matrix
	void pseudoInverseAndNullspace(const Eigen::Matrix<double,3,4>& P, Eigen::Matrix<double,4,3>& Pinv, Eigen::Vector4d& C);

	/// Compute right null-space of A
	Eigen::VectorXd nullspace(const Eigen::MatrixXd& A);

	// Enforce rank deficiency
	Eigen::MatrixXd makeRankDeficient(const Eigen::MatrixXd& A);

	// Compute left null-space of A
	Eigen::VectorXd nullspace_left(const Eigen::MatrixXd& A);
	
} // namespace Geometry

#endif // __singular_value_decomposition_h
