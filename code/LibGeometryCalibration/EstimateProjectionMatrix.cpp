#include "EstimateProjectionMatrix.h"

#include <LibProjectiveGeometry/ProjectiveGeometry.hxx>
#include <LibProjectiveGeometry/ProjectionMatrix.h>
#include <LibProjectiveGeometry/SingularValueDecomposition.h>

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
Geometry::ProjectionMatrix dlt(const std::vector<Eigen::Vector3d>& x, const std::vector<Eigen::Vector4d>& X, const std::map<int,int>& match)
{
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

/// Return value of nearest_match(...)
struct NearestNeighbors {
	std::map<int,int> matching;
	int               number_of_inliers=0;
	double            residual=0;
};

/// Establish matches from a 2D point to the closest projected 3D point, provided that it is no farther than distThreshold away.
NearestNeighbors nearest_match(const std::vector<Geometry::RP2Point>& pts2D, const std::vector<Eigen::Vector2d>& pts3D_P, double distThreshold)
{
	NearestNeighbors ret;
	// Search for every 2D point
	for (int i=0;i<(int)pts2D.size();i++)
	{
		Eigen::Vector2d p=Geometry::euclidian2(pts2D[i]);
		// The closest projected 3D point
		int    min_index=0;
		double min_sq_dist=(pts3D_P.front()-p).norm();
		for (int j=1;j<pts3D_P.size();j++) {
			double sqd=(pts3D_P[j]-p).squaredNorm();
			if (sqd<min_sq_dist) {
				min_sq_dist=sqd;
				min_index=j;
			}
		}
		// And if it is closer than distThreshold, consider it a match
		if (distThreshold*distThreshold>min_sq_dist)
		{
			ret.residual+=std::sqrt(min_sq_dist);
			ret.matching[i]=min_index;
		}
	}
	// Return matches along with mean distance between matching points
	ret.number_of_inliers=(int)ret.matching.size();
	ret.residual/=(double)ret.number_of_inliers;
	return ret;
}

/// 2D Points are assumed to be de-homogenized. pts2D assumed to have homogeneous coordinate 1. See also::nearest_match(...)
NearestNeighbors nearest_match(const Geometry::ProjectionMatrix& P, const std::vector<Geometry::RP2Point>& pts2D, const std::vector<Geometry::RP3Point>& pts3D, double distThreshold)
{
	std::vector<Eigen::Vector2d> pts3D_P(pts3D.size());
	for (int i=0;i<(int)pts3D.size();i++)
		pts3D_P[i]=Geometry::euclidian2(P*pts3D[i]);
	return nearest_match(pts2D,pts3D_P,distThreshold);
}

/// Random Sample Consensus for a projection matrix model using the DLT from point matches. Input assumed to have homogeneous coordinate 1.
Geometry::ProjectionMatrix ransac_projection_matrix(
	const std::map<int,int>&               matching_2d_to_3d,
	const std::vector<Geometry::RP2Point>& detected_beads,
	const std::vector<Geometry::RP3Point>& phantom_beads,
	double                                 inliner_tolerance_px = 4,
	int                                    max_ietartions       = 100,
	double                                 min_inliner_rel      =0.5)
{
	// Currently our best model:
	Geometry::ProjectionMatrix best_P;
	NearestNeighbors           best_model;

	// Try a couple of times (no more than max_ietartions)
	for (int n_iterations=0;n_iterations<max_ietartions;n_iterations++)
	{
		// Choose 6 random indices of point matches
		std::set<int> random_numbers;
		while (random_numbers.size()<6)
			random_numbers.insert(rand()%matching_2d_to_3d.size());
		// Get corresponding matches
		std::map<int,int> minimal_data;
		for (auto it=random_numbers.begin();it!=random_numbers.end();++it) {
			auto mit=matching_2d_to_3d.begin();
			std::advance(mit,*it);
			minimal_data[mit->first]=mit->second;
		}

		// Determine projection matrix from minimal_data
		Geometry::ProjectionMatrix guess=dlt(detected_beads, phantom_beads, minimal_data);

		// Check if we found a better guess than what we had before.
		auto new_model=nearest_match(guess,detected_beads, phantom_beads,inliner_tolerance_px);
		if (new_model.number_of_inliers>best_model.number_of_inliers)
		{
			// We have a new winner
			best_P=guess;
			best_model=new_model;
		}

		// Check if our guess explains the data sufficiently well for an early exit.
		if (best_model.number_of_inliers>min_inliner_rel*phantom_beads.size())
			break;
	}

	// Return projection matrix estimated from all inliers
	return dlt(detected_beads, phantom_beads, best_model.matching);
}

namespace Calibration {

	Geometry::ProjectionMatrix EstimateProjectionMatrix::estimateProjection(
			const std::map<int,int>& matching_2d_to_3d,
			const std::vector<Eigen::Vector3d>& detected_beads,
			const std::vector<Eigen::Vector4d>& phantom_beads
		) const
	{

		inlier_set.clear();

		// Homogenize input (copies everything...)
		std::vector<Geometry::RP2Point> p2d=detected_beads;
		std::vector<Geometry::RP3Point> p3d=phantom_beads;
		for (auto it=p2d.begin();it!=p2d.end();++it) (*it)[2]=1;
		for (auto it=p3d.begin();it!=p3d.end();++it) (*it)[3]=1;

		// Make sure we have even enough matches
		Geometry::ProjectionMatrix P=Geometry::ProjectionMatrix::Zero();
		if (matching_2d_to_3d.size()<6) return P;

		/// Use RANSAC for an initial estimate (or a simple DLT from al matches)
		if (algorithms.use_ransac)
			P=ransac_projection_matrix(
				matching_2d_to_3d,p2d,p3d,
				ransac.inliner_tolerance_px, ransac.max_ietartions, ransac.min_inliner_rel);
		else
			// Plain and simple DLT. Probably a bad idea in the presence of outliers.
			P=dlt(p2d,p3d,matching_2d_to_3d);
		
		/// Use ICP for a refinement (done at least once)
		for (int i=0;i<icp.num_ietartions;i++)
		{
			auto model=nearest_match(P,p2d,p3d,icp.inliner_tolerance_px);
			P=dlt(p2d, p3d, model.matching);
			inlier_set=model.matching; // just for debugging.
		}

		/// 2do gold standard would be optimization of geometric error

		return P;
	}

	void EstimateProjectionMatrix::gui_declare_section (const GetSetGui::Section& section)
	{
		GetSet<int   >("ICP/Num Ietartions"                     , section, icp.num_ietartions                  ).setDescription("The maximum number of iterations allowed in the algorithm.");
		GetSet<double>("ICP/Inliner Tolerance px"               , section, icp.inliner_tolerance_px            ).setDescription("A distance threshold for closest bead for the match to be considered correct.");
		GetSet<int   >("RANSAC/Max Ietartions"                  , section, ransac.max_ietartions               ).setDescription("The maximum number of iterations allowed in the algorithm.");
		GetSet<double>("RANSAC/Inliner Tolerance px"            , section, ransac.inliner_tolerance_px         ).setDescription("When the reprojection error of a point is larger, we consider it an outlier. Also used for ICP matching.");
		GetSet<double>("RANSAC/Min Proportion of Inliners"      , section, ransac.min_inliner_rel              ).setDescription("Good estimates should have a minimum proportion of inliers. Only then can we compare residuals.");
		GetSet<bool  >("Algorithms/Use Random Sample Consensus" , section, algorithms.use_ransac               ).setDescription("Use RANSAC for robust estimation in the presence of false matches.");
		GetSet<bool  >("Algorithms/Use Non Linear Refinement"   , section, algorithms.use_non_linear_refinement).setDescription("Gold standard non-linear optimization of geometric error.");
		GetSetGui::Section     ("ICP"                               , section).setGrouped();
		GetSetGui::Section     ("RANSAC"                            , section).setGrouped();
		GetSetGui::Section     ("Algorithms"                        , section).setGrouped();
		GetSetGui::RangedDouble("RANSAC/Min Proportion of Inliners" , section).setMin(0).setMax(1).setStep(0.01);
	}

	void EstimateProjectionMatrix::gui_retreive_section(const GetSetGui::Section& section)
	{
		icp.num_ietartions                  =GetSet<int   >("ICP/Num Ietartions"                     , section);
		icp.inliner_tolerance_px            =GetSet<double>("ICP/Inliner Tolerance px"               , section);
		ransac.max_ietartions               =GetSet<int   >("RANSAC/Max Ietartions"                  , section);
		ransac.inliner_tolerance_px         =GetSet<double>("RANSAC/Inliner Tolerance px"            , section);
		ransac.min_inliner_rel              =GetSet<double>("RANSAC/Min Proportion of Inliners"      , section);
		algorithms.use_ransac               =GetSet<bool  >("Algorithms/Use Random Sample Consensus" , section);
		algorithms.use_non_linear_refinement=GetSet<bool  >("Algorithms/Use Non Linear Refinement"   , section);
	}

} // namespace Calibration
