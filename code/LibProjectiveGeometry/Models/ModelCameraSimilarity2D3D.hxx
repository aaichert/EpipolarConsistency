#ifndef __model_camera_similarity
#define __model_camera_similarity
// Created by A. Aichert on Fr Sept 12th 2014
// Projective transformation of both world and image P'=T*P*H

#include <LibOpterix/ParameterModel.hxx>
#include "ModelSimilarity2D.hxx"
#include "ModelSimilarity3D.hxx"
#include <LibProjectiveGeometry/ProjectionMatrix.h>

namespace Geometry
{

	// Simple Pinhole Camera Class FIXME how to cleanly access template parameter N of ProjectiveTransormationXD
	class ModelCameraSimilarity2D3D : public LibOpterix::ParameterModel<Geometry::ProjectionMatrix>
	{
		/// The unaltered projection matrix
		Geometry::ProjectionMatrix P;

	public:
		/// Human readable names of all parameters.
		static const std::vector<std::string>& ParameterNames()
		{
			static std::vector<std::string> names;
			if (names.empty())
			{
				// Insert names for ModelSimilarity2D and ModelSimilarity3D separately
				const auto &s2d=ModelSimilarity2D::ParameterNames();
				const auto &s3d=ModelSimilarity3D::ParameterNames();
				names.insert(names.end(), s2d.begin(), s2d.end());
				names.insert(names.end(), s3d.begin(), s3d.end());
			}
			return names;
		}

		/// Frequent sets of active parameters
		static const LibOpterix::ParameterSets& ParameterSets()
		{
			static LibOpterix::ParameterSets sets;
			if (sets.empty())
			{
				// Insert sets for ModelSimilarity2D and adapted indices for sets from ModelSimilarity2D
				int n2d=(int)ModelSimilarity2D::ParameterNames().size();
				const auto &s2d=ModelSimilarity3D::ParameterSets();
				sets.insert(s2d.begin(), s2d.end());
				LibOpterix::ParameterSets s3d=ModelSimilarity2D::ParameterSets();
				for (auto mit=s3d.begin();mit!=s3d.end();++mit)
				{
					std::set<int>& old_idcs(mit->second);
					auto& current_set=sets[mit->first];
					for (auto it=old_idcs.begin();it!=old_idcs.end();++it)
						current_set.insert(*it+n2d);
				}

			}
			return sets;
		}
		
		ModelCameraSimilarity2D3D(Geometry::ProjectionMatrix _P, std::set<int> _active)
			: LibOpterix::ParameterModel<Geometry::ProjectionMatrix>(ParameterNames(),_active)
			, P(_P)
		{}

		/// Update the unaltered projection matrix
		void setOriginalProjectionMatrix(const Geometry::ProjectionMatrix& _P) {
			P=_P;
			std::fill(current_values.begin(), current_values.end(), 0);
		}

		/// The 2D Similarity.
		Geometry::RP2Homography getTransform2D() const
		{
			int n2d=(int)ModelSimilarity2D::ParameterNames().size();
			ModelSimilarity2D m2d;
			m2d.current_values=std::vector<double>(current_values.begin(),current_values.begin()+n2d);
			return m2d.getInstance();
		}

		/// The 3D similarity.
		Geometry::RP3Homography getTransform3D() const
		{
			int n2d=(int)ModelSimilarity2D::ParameterNames().size();
			ModelSimilarity3D m3d;
			m3d.current_values=std::vector<double>(current_values.begin()+n2d,current_values.end());
			return m3d.getInstance();
		}

		/// Transform projection matrix with 2D and 3D similarity transform.
		virtual Geometry::ProjectionMatrix getInstance() const
		{
			return getTransform2D()*P*getTransform3D();
		}
		
	};

} // namespace Geometry

#endif // __camera_transformation
