// Created by A. Aichert on Tue Aug 12th 2014
#ifndef __model_homography2D
#define __model_homography2D

#include <LibOpterix/ParameterModel.hxx>

#include <LibProjectiveGeometry/ProjectiveGeometry.hxx>

namespace Geometry {
	// Over-parametrization for a projective transformation of 2-space
	struct ModelHomography2D : public LibOpterix::ParameterModel<Geometry::RP2Homography>
	{
		/// Human readable names of all parameters.
		static const std::vector<std::string>& ParameterNames()
		{
			static std::vector<std::string> names;
			if (names.empty())
			{
				names.resize(11);
				names[ 0]="Center of Rotation u";
				names[ 1]="Center of Rotation v";
				names[ 2]="Translation u";
				names[ 3]="Translation v";
				names[ 4]="2D Rotation";
				names[ 5]="2D Scale";
				names[ 6]="Scale u";
				names[ 7]="Scale v";
				names[ 8]="Shearing Direction";
				names[ 9]="Projective Distortion u";
				names[10]="Projective Distortion v";
			}
			return names;
		}

		/// Frequent sets of active parameters
		static const LibOpterix::ParameterSets& ParameterSets()
		{
			static LibOpterix::ParameterSets sets;
			if (sets.empty())
			{
				sets["2D Translation"].insert(2);
				sets["2D Translation"].insert(3);
				sets["2D Rigid"].insert(2);
				sets["2D Rigid"].insert(3);
				sets["2D Rigid"].insert(4);
				sets["2D Similarity"].insert(2);
				sets["2D Similarity"].insert(3);
				sets["2D Similarity"].insert(4);
				sets["2D Similarity"].insert(5);
			}
			return sets;
		}
		
		ModelHomography2D(std::set<int> _active)
			: LibOpterix::ParameterModel<Geometry::RP2Homography>(ParameterNames(),_active)
		{}
		
		// Compose a 2D homography from parametrization
		virtual Geometry::RP2Homography getInstance() const
		{
			using namespace Eigen;
			const std::vector<double>& x(current_values);

			Matrix3d H=Matrix3d::Identity();
			// Rotation
			if (x[4]!=0)
			{
				H <<	+cos(x[4]),	-sin(x[4]),	0,
						+sin(x[4]),	+cos(x[4]),	0,
						0,			0,			1;
			}
			// Center of rotation
			Eigen::Vector3d c=H*Eigen::Vector3d(x[0],x[1],1);
			// Translation
			H(0,2)=c(0)-x[0]+x[2];
			H(1,2)=c(1)-x[1]+x[3];
			// (Anisotropic) scaling
			Matrix3d D=Eigen::Vector3d((1.0+x[5])*(1.0+x[6]),(1.0+x[5])*(1.0+x[7]),1.0).asDiagonal();
			if (x[8])
			{
				// Shearing
				Matrix3d Rs;
				Rs <<	+cos(x[8]),	-sin(x[8]),	0,
						+sin(x[8]),	+cos(x[8]),	0,
						0,			0,			1;
				// Apply shearing: T=Rigid*Rs'*D*Rs;
				H=H*Rs.transpose()*D*Rs;
			}
			else
				H=H*D;
			// Projective Distortion
			H(2,0)=x[9];
			H(2,1)=x[10];
			// Return result
			return H;
		}
		
	};

} // namespace Geometry 

#endif // __model_homography2D
