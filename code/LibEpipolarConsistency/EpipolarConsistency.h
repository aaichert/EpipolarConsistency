#ifndef __epipolar_consistency
#define __epipolar_consistency
// Created by A. Aichert on Thu Nov 21st 2013
// Library to compute a metric of epipolar consistency.
// Several different algorithms are available.
// 1) Epipolar Conistency for motion and calibration correction.
//    See EpipolarConsistencyRadonIntermediate.h:
//    This version pre-computes the Radon intermediate function allowing for a
//    much faster evaluation, assuming that the projection images do not change.
//    It also supports using a ramp-filter instead of the usual derivative.
// 2) Epipolar Consistency for projection image correction (cupping, scatter etc...)
//    See also: EpipolarConsistencyDirect.h:
//    This version is preferred when the metric is evaluated only once, or, if
//    the projection images are subject to change. It also supports the evaluation
//    of fan-beam consistency conditions by rectification (no derivative).


// Projective Geometry
#include <LibProjectiveGeometry/ProjectionMatrix.h>

#include <vector>

// Predeclaration of some CUDA stuff
namespace UtilsCuda {
	template <typename T> class BindlessTexture2D;
	template <typename T> class MemoryBlock;
} // namespace UtilsCuda

namespace EpipolarConsistency
{
	using Geometry::Pi;
	using Geometry::RP3Point;
	using Geometry::RP3Line;
	using Geometry::ProjectionMatrix; 

	/// Compute point with least squared distance to all principal rays.
	RP3Point estimateIsoCenter(const std::vector<ProjectionMatrix> &Ps);
	
	/// Compute radius of sphere around object, assuming that the object is centered around the origin and P is facing it directly.
	double estimateObjectRadius(const ProjectionMatrix& P, int n_u, int n_v);

	/// Range of epipolar plane angle kappa for two projections, assuming the object is bounded by a sphere with object_radius_mm around origin.
	std::pair<double,double> estimateAngularRange(const RP3Line& B, double object_radius_mm);

	/// A good step of epipolar plane angle kappa, assuming that the object is centered around the origin and P is facing it directly.
	double estimateAngularStep(const ProjectionMatrix& P0, const ProjectionMatrix& P1, int n_u, int n_v);

	/// Interface for Epipolar Consistency metric given projection matrices and projection images.
	class Metric {
		/// Radius of object. Determines range for which planes are sampled. Zero for automatic.
		double object_radius_mm;
	protected:
		/// Angle between epipolar planes during sampling. Zero for automatic.
		double dkappa; 
		std::vector<ProjectionMatrix> Ps;	//< Current projection matrices
		int n_u;							//< Image size
		int n_v;							//< Image size

	public:
		/// Set radius of object.  Zero for automatic. Sampling occurs for all planes which intersect the sphere with that radius. 
		Metric& setObjectRadius(double radius_mm=0);

		/// Sampling occurs for all planes which intersect the sphere with that radius. 
		double getObjectRadius() const;

		/// Manually angle between epipolar planes during sampling. Zero for automatic determination per view pair (recommended).
		Metric& setEpipolarPlaneStep(double dkappa_rad=0);
		
		/// Set projection matrices.
		virtual Metric& setProjectionMatrices(const std::vector<ProjectionMatrix>& Ps);

		/// Get projection matrices.
		const std::vector<ProjectionMatrix>& getProjectionMatrices() const;

		/// Set projections images from single-channel 2D float textures.
		virtual Metric& setProjectionImages(const std::vector<UtilsCuda::BindlessTexture2D<float>*>& Is) = 0;

		/// The number of projections. The number of evaluations will be n*(n-1)/2
		virtual int  getNumberOfProjetions() = 0;

		/// Evaluates metric and optionally returns n*n cost image.
		virtual double evaluate(float *cost_image=0x0) = 0;

		/// Evaluate for just tow images i and j and optionally also return redundant values.
		virtual double evaluateForImagePair(int i, int j,
			std::vector<float> *redundant_samples0=0x0, std::vector<float> *redundant_samples1=0x0,
			std::vector<float> *kappas=0x0) =0;

		/// Constructor
		Metric();

		/// Virtual destructor.
		virtual ~Metric();
	};


} // namespace EpipolarConsistency

#endif // __epipolar_consistency
