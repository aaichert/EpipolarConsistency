#ifndef __epipolar_consistency_direct_h
#define __epipolar_consistency_direct_h
// Created by A. Aichert on Thu Nov 21st 2018
// Class to compute Epipolar Consistency given ProjectionMatrices and pre-processed x-ray proections.

#include "EpipolarConsistency.h"

// NRRD Image File Format
#include <NRRD/nrrd_image.hxx>

// Projective Geometry
#include <LibProjectiveGeometry/ProjectionMatrix.h>

namespace EpipolarConsistency
{
	/// The main algorithm behind epipolar consistency, when not using Radon intermediate functions.
	double computeForImagePair(
		const Geometry::ProjectionMatrix&          P0, const Geometry::ProjectionMatrix&          P1,
		const UtilsCuda::BindlessTexture2D<float>& I0, const UtilsCuda::BindlessTexture2D<float>& I1,
		double dkappa, double object_radius_mm, bool fbcc=false,
		std::vector<float> *redundant_samples0=0x0, std::vector<float> *redundant_samples1=0x0,
		std::vector<float> *kappas=0x0);
		
	/// How epipolar planes will be sampled
	// enum SamplingStrategy { RegularEpipolarAngle, EqualNumberOfSamples } 

	/// Compute Epipolar Consistency on the GPU directly from projection images.
	class MetricDirect : public Metric {
		/// Projection matrices
		std::vector<ProjectionMatrix> Ps;
		///  Projection images as 2D single.channel float Cuda textures.
		std::vector<UtilsCuda::BindlessTexture2D<float>*> Is;
		/// Use standard epipolar consistecy with derivative or the rectified version without derivative?
		bool use_fbcc;
	public:

		/// Direct evaluation of epipolar consistency metrix (for repeated evaluations see also: MetricRadonIntermediate)
		MetricDirect(const std::vector<ProjectionMatrix>& Ps, const std::vector<UtilsCuda::BindlessTexture2D<float>*>& _Is);

		/// Set projection matrices.
		virtual Metric& setProjectionMatrices(const std::vector<ProjectionMatrix>& Ps);

		/// Set projections images from single-channel 2D float textures.
		virtual Metric& setProjectionImages(const std::vector<UtilsCuda::BindlessTexture2D<float>*>& Is);

		/// The number of projections. The number of evaluations will be n*(n-1)/2
		virtual int  getNumberOfProjetions();

		/// Evaluates metric and optionally returns n*n cost image.
		virtual double evaluate(float * out=0x0);

		/// Evaluate for just tow images i and j and optionally also return redundant values.
		virtual double evaluateForImagePair(int i, int j,
			std::vector<float> *redundant_samples0=0x0, std::vector<float> *redundant_samples1=0x0,
			std::vector<float> *kappas=0x0);

		/// Change algorith to use retification instead of derivative.
		MetricDirect& setFanBeamConsistency(bool fbcc=true);

	};

} // namespace EpipolarConsistency

#endif // __epipolar_consistency_direct_h
