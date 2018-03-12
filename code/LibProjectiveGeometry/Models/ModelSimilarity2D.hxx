// Created by A. Aichert on Tue Aug 12th 2014
#ifndef __model_similarity2D
#define __model_similarity2D

#include <LibOpterix/ParameterModel.hxx>

#include <LibProjectiveGeometry/ProjectiveGeometry.hxx>

namespace Geometry {
	// Parametrization for a similarity transformation of 2-space
	struct ModelSimilarity2D : public LibOpterix::ParameterModel<Geometry::RP2Homography>
	{
		/// Human readable names of all parameters.
		static const std::vector<std::string>& ParameterNames()
		{
			static std::vector<std::string> names;
			if (names.empty())
			{
				names.resize(4);
				names[ 0]="Translation u";
				names[ 1]="Translation v";
				names[ 2]="2D Rotation";
				names[ 3]="2D Scale";
			}
			return names;
		}

		/// Frequent sets of active parameters
		static const LibOpterix::ParameterSets& ParameterSets()
		{
			static LibOpterix::ParameterSets sets;
			if (sets.empty())
			{
				sets["2D Translation"].insert(0);
				sets["2D Translation"].insert(1);
				sets["2D Rigid"].insert(0);
				sets["2D Rigid"].insert(1);
				sets["2D Rigid"].insert(3);
				sets["2D Similarity"].insert(0);
				sets["2D Similarity"].insert(1);
				sets["2D Similarity"].insert(2);
				sets["2D Similarity"].insert(3);
			}
			return sets;
		}

		ModelSimilarity2D(std::set<int> _active=std::set<int>())
			: LibOpterix::ParameterModel<Geometry::RP2Homography>(ParameterNames(),_active)
		{}

		// Compose a 2D homography from parametrization
		virtual Geometry::RP2Homography getInstance() const
		{
			using namespace Eigen;
			const std::vector<double>& x(current_values);

			Matrix3d H=Matrix3d::Identity();
			// Rotation
			if (x[2]!=0)
			{
				H <<	+cos(x[2]),	-sin(x[2]),	0,
						+sin(x[2]),	+cos(x[2]),	0,
						0,			0,			1;
			}
			// Translation
			H(0,2)=+x[0];
			H(1,2)=+x[1];
			// Apply scaling
			if (x[3]!=0) H.block<2,2>(0,0)*=(1.0+x[3]);
			// Return result
			return H;
		}
		
	};

} // namespace Geometry 

#endif // __model_similarity2D
