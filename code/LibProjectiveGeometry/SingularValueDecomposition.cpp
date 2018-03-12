
#include <Eigen/SVD>

using namespace Eigen;
	
namespace Geometry
{

	/// Compute the pseudo-inverse of a matrix
	Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd& A)
	{
		JacobiSVD<MatrixXd> svd(A, ComputeFullU | ComputeFullV);
		double  tolerance=1e-6;
		int u=(int)A.rows(), v=(int)A.cols(), n=std::min<int>(u,v);
		MatrixXd sigma(v,u);
		sigma.setZero();
		sigma.topLeftCorner(n,n)=svd.singularValues().asDiagonal();
		for (int i=0;i<n;i++)
			if (sigma(i,i)<tolerance) sigma(i,i)=0;
			else sigma(i,i)=1.0/sigma(i,i);
		MatrixXd pinv=svd.matrixV()*sigma*svd.matrixU().transpose();
		return pinv;
	}

	/// Compute the pseudo-inverse and null-space of a matrix
	void pseudoInverseAndNullspace(const Eigen::Matrix<double,3,4>& P, Eigen::Matrix<double,4,3>& Pinv, Eigen::Vector4d& C)
	{
		JacobiSVD<MatrixXd> svd(P, ComputeFullU | ComputeFullV);
		double  tolerance=1e-6;
		int u=(int)P.rows(), v=(int)P.cols(), n=std::min<int>(u,v);
		MatrixXd sigma(v,u);
		sigma.setZero();
		sigma.topLeftCorner(n,n)=svd.singularValues().asDiagonal();
		for (int i=0;i<n;i++)
			if (sigma(i,i)<tolerance) sigma(i,i)=0;
			else sigma(i,i)=1.0/sigma(i,i);
		Pinv=svd.matrixV()*sigma*svd.matrixU().transpose();
		auto V=svd.matrixV();
		C=V.col(V.cols()-1);
	}

	/// Compute right null-space of A
	Eigen::VectorXd nullspace(const Eigen::MatrixXd& A)
	{
		JacobiSVD<MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
		auto V=svd.matrixV();
		return V.col(V.cols()-1);
	}

	// Enforce rank deficiency
	Eigen::MatrixXd makeRankDeficient(const Eigen::MatrixXd& A)
	{
		JacobiSVD<MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
		VectorXd sigma=svd.singularValues();
		sigma[sigma.size()-1]=0;
		return svd.matrixU()*sigma.asDiagonal()*svd.matrixV().transpose();
	}
	
	// Compute left null-space of A
	Eigen::VectorXd nullspace_left(const Eigen::MatrixXd& A)
	{
		return nullspace(A.transpose());
	}
	
} // namespace Geometry
