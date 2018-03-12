// Created by A. Aichert on Tue Aug 12th 2014
#ifndef __model_similarity3D
#define __model_similarity3D

#include <LibOpterix/ParameterModel.hxx>

#include <LibProjectiveGeometry/ProjectiveGeometry.hxx>

#include <Eigen/Geometry>


namespace Geometry {
	// Parametrization for a similarity transformation of 3-space
	struct ModelSimilarity3D : public LibOpterix::ParameterModel<Geometry::RP3Homography>
	{
		/// Human readable names of all parameters.
		static const std::vector<std::string>& ParameterNames()
		{
			static std::vector<std::string> names;
			if (names.empty())
			{
				names.resize(7);
				names[ 0]="Translation X";
				names[ 1]="Translation Y";
				names[ 2]="Translation Z";
				names[ 3]="Rotation X";
				names[ 4]="Rotation Y";
				names[ 5]="Rotation Z";
				names[ 6]="3D Scale";
			}
			return names;
		}

		/// Frequent sets of active parameters
		static const LibOpterix::ParameterSets& ParameterSets()
		{
			static LibOpterix::ParameterSets sets;
			if (sets.empty())
			{
				sets["3D Translation"].insert(0);
				sets["3D Translation"].insert(1);
				sets["3D Translation"].insert(2);
				sets["3D Rigid"].insert(0);
				sets["3D Rigid"].insert(1);
				sets["3D Rigid"].insert(2);
				sets["3D Rigid"].insert(3);
				sets["3D Rigid"].insert(4);
				sets["3D Rigid"].insert(5);
				sets["3D Similarity"].insert(0);
				sets["3D Similarity"].insert(1);
				sets["3D Similarity"].insert(2);
				sets["3D Similarity"].insert(3);
				sets["3D Similarity"].insert(4);
				sets["3D Similarity"].insert(5);
				sets["3D Similarity"].insert(6);
			}
			return sets;
		}
		

		ModelSimilarity3D(std::set<int> _active=std::set<int>())
			: LibOpterix::ParameterModel<Geometry::RP3Homography>(ParameterNames(),_active)
		{}

		// Compose a 3D homography from parametrization
		virtual Geometry::RP3Homography getInstance() const
		{
			using namespace Eigen;
			// Slicker naming of the parameter vector.
			const std::vector<double>& x(current_values);
			// We begin with an identity
			Matrix4d T=Matrix4d::Identity();
			// Rotation about X Y and Z
			if (x[3]!=0||x[4]!=0||x[5]!=0)
			{
				auto R=( AngleAxisd(x[3], Vector3d::UnitX())
						*AngleAxisd(x[4], Vector3d::UnitY())
						*AngleAxisd(x[5], Vector3d::UnitZ()));
				T.block<3,3>(0,0)=(Matrix3d)R;
			}
			// Center of rotation and translation
			T(0,3)=x[0];
			T(1,3)=x[1];
			T(2,3)=x[2];
			// Apply scaling
			if (x[6]!=0) T.block<3,3>(0,0)*=(1.0+x[6]);
			// Return result
			return T;
		}
		
	};

} // namespace Geometry 

#endif // __model_similarity3D
