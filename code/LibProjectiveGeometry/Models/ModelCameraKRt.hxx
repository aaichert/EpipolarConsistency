#ifndef __model_camera_decomposition
#define __model_camera_decomposition
// Created by A. Aichert on Fr July 27th 2018

#include <LibOpterix/ParameterModel.hxx>
#include <LibProjectiveGeometry/ProjectionMatrix.h>

namespace Geometry
{

	// Simple Pinhole Camera Class from intrinsic and extrinsic parameters
	class ModelCameraKRt : public LibOpterix::ParameterModel<Geometry::ProjectionMatrix>
	{
	public:
		/// Human readable names of all parameters.
		static const std::vector<std::string>& ParameterNames()
		{
			static std::vector<std::string> names;
			if (names.empty())
			{
				names.push_back("Focal Length u [px]");
				names.push_back("Focal Length v [px]");
				names.push_back("Principal Point u [px]");
				names.push_back("Principal Point v [px]");
				names.push_back("Skew");
				names.push_back("Translation X");
				names.push_back("Translation Y");
				names.push_back("Translation Z");
				names.push_back("Rotation X");
				names.push_back("Rotation Y");
				names.push_back("Rotation Z");
			}
			return names;
		}

		/// Frequent sets of active parameters
		static const LibOpterix::ParameterSets& ParameterSets()
		{
			static LibOpterix::ParameterSets sets;
			if (sets.empty())
			{
				
				// 2do

			}
			return sets;
		}
		
		ModelCameraKRt(std::set<int> _active = std::set<int>())
			: LibOpterix::ParameterModel<Geometry::ProjectionMatrix>(ParameterNames(),_active)
		{}

		/// Transform projection matrix with 2D and 3D similarity transform.
		virtual Geometry::ProjectionMatrix getInstance() const
		{
			const std::vector<double>& x(current_values);
			Eigen::Matrix3d K,R;
			Eigen::Vector3d t;

			K << x[0], x[4], x[2],
				    0, x[1], x[3],
				    0,   0 ,   1;
			
			R = (RotationX(x[8])*RotationY(x[9])*RotationZ(x[10])).block<3,3>(0,0);
			t[0]=x[5];
			t[1]=x[6];
			t[2]=x[7];

			return makeProjectionMatrix(K,R,t);
		}
		
		/// Decompose existing projection matrix
		virtual ModelCameraKRt& setInstance(const Geometry::ProjectionMatrix&  P, bool v_points_up)
		{
			Eigen::Matrix3d K,R;
			Eigen::Vector3d t;
			Geometry::projectionMatrixDecomposition(P,K,R,t,v_points_up);
			std::vector<double>& x(current_values);
			x[ 0]=K(0,0);
			x[ 1]=K(1,1);
			x[ 2]=K(0,2);
			x[ 3]=K(1,2);
			x[ 4]=K(0,1);
			x[ 5]=t(0);
			x[ 6]=t(1);
			x[ 7]=t(2);
			x[ 8]=std::asin(R(1,3));
			x[ 9]=std::acos(R(1,1)/cos(x[ 8]));
			x[10]=std::acos(R(3,3)/cos(x[ 8]));
			return *this;
		}
	};

} // namespace Geometry

#endif // __model_camera_decomposition
