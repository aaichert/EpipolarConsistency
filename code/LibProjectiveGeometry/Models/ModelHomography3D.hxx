// Created by A. Aichert on Tue Aug 12th 2014
#ifndef __model_homography3D
#define __model_homography3D

#include <LibOpterix/ParameterModel.hxx>

#include <LibProjectiveGeometry/ProjectiveGeometry.hxx>

namespace Geometry {
	// Over-parametrization for a projective transformation of 3-space
	struct ModelHomography3D : public LibOpterix::ParameterModel<Geometry::RP3Homography>
	{
		/// Human readable names of all parameters.
		static const std::vector<std::string>& ParameterNames()
		{
			static std::vector<std::string> names;
			if (names.empty())
			{
				names.resize(19);
				names[ 0]="Center of Rotation X";
				names[ 1]="Center of Rotation Y";
				names[ 2]="Center of Rotation Z";
				names[ 3]="Translation X";
				names[ 4]="Translation Y";
				names[ 5]="Translation Z";
				names[ 6]="Rotation X";
				names[ 7]="Rotation Y";
				names[ 8]="Rotation Z";
				names[ 9]="3D Scale";
				names[10]="Scale X";
				names[11]="Scale Y";
				names[12]="Scale Z";
				names[13]="Shearing Direction X";
				names[14]="Shearing Direction Y";
				names[15]="Shearing Direction Z";			
				names[16]="Projective Distortion X";
				names[17]="Projective Distortion Y";
				names[18]="Projective Distortion Z";
			}
			return names;
		}

		/// Frequent sets of active parameters
		static const LibOpterix::ParameterSets& ParameterSets()
		{
			static LibOpterix::ParameterSets sets;
			if (sets.empty())
			{
				sets["3D Translation"].insert(3);
				sets["3D Translation"].insert(4);
				sets["3D Translation"].insert(5);
				sets["3D Rigid"].insert(3);
				sets["3D Rigid"].insert(4);
				sets["3D Rigid"].insert(5);
				sets["3D Rigid"].insert(6);
				sets["3D Rigid"].insert(7);
				sets["3D Rigid"].insert(8);
				sets["3D Similarity"].insert(3);
				sets["3D Similarity"].insert(4);
				sets["3D Similarity"].insert(5);
				sets["3D Similarity"].insert(6);
				sets["3D Similarity"].insert(7);
				sets["3D Similarity"].insert(8);
				sets["3D Similarity"].insert(10);
				sets["3D Similarity"].insert(11);
				sets["3D Similarity"].insert(12);
			}
			return sets;
		}
		
		ModelHomography3D(std::set<int> _active)
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
			auto R=( AngleAxisd(x[6], Vector3d::UnitX())
					*AngleAxisd(x[7], Vector3d::UnitY())
					*AngleAxisd(x[8], Vector3d::UnitZ()));
			T.block<3,3>(0,0)=(Matrix3d)R;
			// Center of rotation
			Eigen::Vector3d c=R*Eigen::Vector3d(x[0],x[1],x[2]);
			// Center of rotation and translation
			T(0,3)=c(0)-x[0]+x[3];
			T(1,3)=c(1)-x[1]+x[4];
			T(2,3)=c(2)-x[2]+x[5];
			// (Anisotropic) scaling
			Matrix4d D=Eigen::Vector4d(x[9]*x[10],x[9]*x[11],x[9]*x[12],1.0).asDiagonal();
			if (x[13]||x[14]||x[15])
			{
				// Shearing
				auto Rsq=(AngleAxisd(x[13], Vector3d::UnitX())
						 *AngleAxisd(x[14], Vector3d::UnitY())
						 *AngleAxisd(x[15], Vector3d::UnitZ()));
				Matrix4d Rs=Matrix4d::Identity();
				Rs.block<3,3>(0,0)=(Matrix3d)Rsq;
				// Apply shearing: T=Rigid*Rs'*D*Rs;
				T=T*Rs.inverse()*D*Rs;
			}
			else
				T=T*D;
			// Projective Distortion
			T(3,0)=x[16];
			T(3,1)=x[17];
			T(3,2)=x[18];
			// Return result
			return T;
		}
		
	};

} // namespace Geometry 

#endif // __model_homography3D
