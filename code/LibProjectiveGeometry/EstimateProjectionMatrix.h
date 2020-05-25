#ifndef __direct_linear_transformation
#define __direct_linear_transformation

#include "ProjectiveGeometry.hxx"

#include <vector>
#include <map>

namespace Geometry {

	/// Direct Linear Transformation for projection matrices. Input assumed to have homogeneous coordinate 1.
	Geometry::ProjectionMatrix dlt(const std::vector<Eigen::Vector3d>& x, const std::vector<Eigen::Vector4d>& X, std::map<int,int> match=std::map<int,int>());

} // namespace Geometry

#endif // __direct_linear_transformation
